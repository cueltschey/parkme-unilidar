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
                                     float lidar_z)
    : sequence_id_(0),
      lidar_x_(lidar_x), lidar_y_(lidar_y), lidar_z_(lidar_z),
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

    struct Placement { const char* path; float x, y, z, yaw; };

    // Four hatchback cars. Scale 0.0254 converts the OBJ from inches → metres;
    // no pitch correction needed — the mesh is already Z-up in its own frame.
    Placement cars[] = {
        {"meshes/hatchback.obj",  5.0f,  0.0f,  0.0f,  0.0f},
        {"meshes/hatchback.obj", -5.0f,  0.0f,  0.0f,  (float)( M_PI)},
        {"meshes/hatchback.obj",  0.0f,  5.0f,  0.0f,  (float)( M_PI / 2.0)},
        {"meshes/hatchback.obj",  0.0f, -5.0f,  0.0f,  (float)(-M_PI / 2.0)},
    };
    for (auto& p : cars) {
        scene_object obj;
        if (create_mesh_from_file(p.path, &obj, 0.0254f, 0.0254f, 0.0254f)) {
            obj.transform = create_transformation_matrix(
                p.x, p.y, p.z, 0.0f, 0.0f, p.yaw, "XYZ");
            scene_add_object(&scene_->scn, obj);
        } else {
            LOG_WARNING("Failed to load mesh: %s", p.path);
        }
    }

    // Three trees — no pitch correction needed (trees are upright in OBJ).
    Placement trees[] = {
        //{"meshes/tree.obj",  2.0f,  5.0f, 0.0f, 0.0f},
        //{"meshes/tree.obj", -3.0f, -4.0f, 0.0f, 0.0f},
        //{"meshes/tree.obj",  6.0f,  0.0f, 0.0f, 0.0f},
    };
    for (auto& p : trees) {
        scene_object obj;
        if (create_mesh_from_file(p.path, &obj, 1.0f, 1.0f, 1.0f)) {
            obj.transform = create_transformation_matrix(
                p.x, p.y, p.z, 0.0f, 0.0f, p.yaw, "ZY");
            scene_add_object(&scene_->scn, obj);
        } else {
            LOG_WARNING("Failed to load mesh: %s", p.path);
        }
    }

    // Ground plane centred at the origin — large enough to undercut all cars.
    // Rays that pass below the car body (no undercarriage geometry in the mesh)
    // hit this surface instead of returning empty, completing the silhouette.
    scene_object ground = create_plane(40.0f, 40.0f);
    ground.transform = create_transformation_matrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, "XYZ");
    scene_add_object(&scene_->scn, ground);

    // Separate plane at (50, 50) — flat in the XY plane, 20 × 20 units.
    scene_object plane = create_plane(20.0f, 20.0f);
    plane.transform = create_transformation_matrix(50.0f, 50.0f, 0.0f, 0.0f, 0.0f, 0.0f, "XYZ");
    scene_add_object(&scene_->scn, plane);

    // Apply lidar-relative transforms and build the top-level BVH.
    scene_update(&scene_->scn, scene_->ldr);
    scene_build(&scene_->scn);

    LOG_INFO("EmulationScene ready: %d objects loaded", scene_->scn.current_size);
}

void EmulationProducer::produce()
{
    while (running_)
    {
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
