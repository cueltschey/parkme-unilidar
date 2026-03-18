#include "emulation_producer.h"
#include "lidarsim.h"
#include "logger.h"
#include <chrono>
#include <float.h>

// Holds the lidarsim scene and lidar objects. Defined here so lidarsim.h is
// only compiled into this translation unit.
struct EmulationScene {
    scene scn;
    lidar ldr;
};

EmulationProducer::EmulationProducer(uint32_t /*num_rings*/,
                                     uint32_t /*points_per_ring*/,
                                     float lidar_x,
                                     float lidar_y,
                                     float lidar_z,
                                     double scene_duration)
    : sequence_id_(0),
      lidar_x_(lidar_x), lidar_y_(lidar_y), lidar_z_(lidar_z),
      scene_duration_(scene_duration),
      last_scene_change_(std::chrono::steady_clock::now()),
      rng_(std::random_device{}()),
      scene_(std::make_unique<EmulationScene>())
{
    initScene();
    production_thread_ = std::thread(&EmulationProducer::produce, this);
}

EmulationProducer::~EmulationProducer() {
    running_ = false;
    if (production_thread_.joinable()) {
        production_thread_.join();
    }
    scene_free(&scene_->scn);
    lidar_free(&scene_->ldr);
}

void EmulationProducer::initScene() {
    scene_init(&scene_->scn);

    // Load the mid360 ray pattern from CSV.
    if (!create_lidar_from_file("csv/mid360.csv", &scene_->ldr)) {
        LOG_ERROR("Failed to load lidar rays from csv/mid360.csv");
    } else {
        LOG_INFO("Loaded %u rays from mid360.csv", scene_->ldr.ray_count);
    }

    // Position the lidar as specified by config (default: 1 m above origin).
    scene_->ldr.transform = create_transformation_matrix(
        lidar_x_, lidar_y_, lidar_z_, 0.0f, 0.0f, 0.0f, "XYZ");
    LOG_INFO("LiDAR position: (%.2f, %.2f, %.2f)", lidar_x_, lidar_y_, lidar_z_);

    randomizeScene();
}

void EmulationProducer::randomizeScene() {
    // Clear existing objects before rebuilding.
    scene_free(&scene_->scn);
    scene_init(&scene_->scn);

    // Random car count: 2–8.
    std::uniform_int_distribution<int> count_dist(2, 8);
    int num_cars = count_dist(rng_);

    // Cars are placed within a 12 m radius; minimum 4 m separation between centres.
    const float MAX_RADIUS = 12.0f;
    const float MIN_SEP    =  4.0f;
    // Yaw snapped to cardinal directions.
    const float yaws[] = { 0.0f, (float)(M_PI / 2.0), (float)(M_PI), (float)(3.0 * M_PI / 2.0) };

    std::uniform_real_distribution<float> angle_dist(0.0f, (float)(2.0 * M_PI));
    std::uniform_real_distribution<float> radius_dist(3.0f, MAX_RADIUS);
    std::uniform_int_distribution<int>    yaw_dist(0, 3);

    struct CarPos { float x, y; };
    std::vector<CarPos> placed;
    placed.reserve(num_cars);

    int attempts = 0;
    while ((int)placed.size() < num_cars && attempts < 200) {
        ++attempts;
        float angle = angle_dist(rng_);
        float r     = radius_dist(rng_);
        float cx = r * std::cos(angle);
        float cy = r * std::sin(angle);

        // Reject if too close to any already-placed car.
        bool ok = true;
        for (const auto& p : placed) {
            float dx = cx - p.x, dy = cy - p.y;
            if (std::sqrt(dx*dx + dy*dy) < MIN_SEP) { ok = false; break; }
        }
        if (!ok) continue;

        float yaw = yaws[yaw_dist(rng_)];
        scene_object obj;
        if (create_mesh_from_file("meshes/hatchback.obj", &obj, 0.0254f, 0.0254f, 0.0254f)) {
            obj.transform = create_transformation_matrix(cx, cy, 0.0f, 0.0f, 0.0f, yaw, "XYZ");
            scene_add_object(&scene_->scn, obj);
            placed.push_back({cx, cy});
        } else {
            LOG_WARNING("Failed to load mesh: meshes/hatchback.obj");
            break; // no point retrying if the file is missing
        }
    }
    LOG_INFO("Randomized scene: %d cars placed", (int)placed.size());

    // Ground plane — always present so wheel-well gaps are filled.
    scene_object ground = create_plane(40.0f, 40.0f);
    ground.transform = create_transformation_matrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, "XYZ");
    scene_add_object(&scene_->scn, ground);

    // Apply lidar-relative transforms and rebuild the top-level BVH.
    scene_update(&scene_->scn, scene_->ldr);
    scene_build(&scene_->scn);

    last_scene_change_ = std::chrono::steady_clock::now();
    LOG_INFO("EmulationScene ready: %d objects loaded", scene_->scn.current_size);
}

void EmulationProducer::produce()
{
    while (running_)
    {
        // Periodically randomize the scene.
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_scene_change_).count();
        if (elapsed >= scene_duration_) {
            randomizeScene();
        }

        unilidar_sdk2::PointCloudUnitree cloud;

        cloud.stamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        cloud.id      = sequence_id_;
        cloud.ringNum = 1;
        cloud.points.clear();
        cloud.points.reserve(scene_->ldr.ray_count);

        for (uint32_t i = 0; i < scene_->ldr.ray_count; ++i)
        {
            ray& r = scene_->ldr.rays[i];

            ray_result closest;
            closest.t   = FLT_MAX;
            closest.obj = nullptr;

            // The top-level BVH (bvh_traverse_ray_aabb) stores leaf indices as
            // positions in the qsort_r-sorted pointer array, but then looks them
            // up in the original scn->objects array — a mismatch that silently
            // skips non-mesh objects like planes.  Iterating objects directly and
            // using each object's own AABB as a cheap pre-reject gives correct
            // results for all object types with no per-frame allocation cost.
            for (int obj_idx = 0; obj_idx < scene_->scn.current_size; ++obj_idx) {
                scene_object* obj = &scene_->scn.objects[obj_idx];
                float t_aabb;
                if (!ray_aabb(&r, &obj->aabb, &t_aabb) || t_aabb >= closest.t)
                    continue;
                check_and_update_intersection(&r, obj, &closest);
            }

            unilidar_sdk2::PointUnitree point{};
            if (closest.obj != nullptr) {
                point.x         = r.ori.x + closest.t * r.dir.x;
                point.y         = r.ori.y + closest.t * r.dir.y;
                point.z         = r.ori.z + closest.t * r.dir.z;
                point.intensity = 1.0f;
            }
            point.ring = 0;
            point.time = 0.0;
            cloud.points.push_back(point);
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            cloud_queue_.push(std::move(cloud));
        }
        queue_cv_.notify_one();
        sequence_id_++;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
