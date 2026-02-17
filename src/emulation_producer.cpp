#include "emulation_producer.h"
#include <iostream>
#include "logger.h"

EmulationProducer::EmulationProducer(const std::string& obj_path,
                                     uint32_t num_rings,
                                     uint32_t points_per_ring)
    : num_rings_(num_rings),
      points_per_ring_(points_per_ring),
      sequence_id_(0)
{
    if (!loadOBJ(obj_path)) {
        std::cerr << "Failed to load OBJ: " << obj_path << std::endl;
    }

    production_thread_ = std::thread(&EmulationProducer::produce, this);
}



bool EmulationProducer::loadOBJ(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string prefix;
        ss >> prefix;

        if (prefix == "v") {
            Vec3 v;
            ss >> v.x >> v.y >> v.z;
            vertices_.push_back(v);
        }
	else if (prefix == "f") {
	    std::string v0_str, v1_str, v2_str;
	    ss >> v0_str >> v1_str >> v2_str;

	    auto parseIndex = [](const std::string& token) -> int {
		size_t slash_pos = token.find('/');
		if (slash_pos == std::string::npos)
		    return std::stoi(token);
		return std::stoi(token.substr(0, slash_pos));
	    };

	    int i0 = parseIndex(v0_str);
	    int i1 = parseIndex(v1_str);
	    int i2 = parseIndex(v2_str);

	    if (i0 <= 0 || i1 <= 0 || i2 <= 0 ||
		i0 > vertices_.size() ||
		i1 > vertices_.size() ||
		i2 > vertices_.size())
	    {
		LOG_DEBUG("unsafe vert");
		continue;
	    }

	    Triangle tri;
	    tri.v0 = vertices_[i0 - 1];
	    tri.v1 = vertices_[i1 - 1];
	    tri.v2 = vertices_[i2 - 1];

	    triangles_.push_back(tri);
	}
    }

    std::cout << "Loaded OBJ: "
              << vertices_.size() << " vertices, "
              << triangles_.size() << " triangles\n";

    return true;
}

unilidar_sdk2::PointUnitree
EmulationProducer::generatePoint(double angle_horizontal,
                                 double angle_vertical)
{
    unilidar_sdk2::PointUnitree point{};

    // Ray direction
    Vec3 dir;
    dir.x = cos(angle_vertical) * cos(angle_horizontal);
    dir.y = cos(angle_vertical) * sin(angle_horizontal);
    dir.z = sin(angle_vertical);

    //point.x = dir.x;
    //point.y = dir.y;
    //point.z = dir.z;
    //point.intensity = 0.2;

    Vec3 hit;

    if (dir.z < 0.0) {
        double t = -0.0 / dir.z;
        point.x = dir.x * t;
        point.y = dir.y * t;
        point.z = 0.0;
        point.intensity = 0.5f;
    }
    if (intersectMesh(dir, hit)) {
        point.x = hit.x;
        point.y = hit.y;
        point.z = hit.z;
        point.intensity = 1.0f;
    }

    point.time = 0.0;
    return point;
}


bool EmulationProducer::intersectMesh(const Vec3& ray_dir,
                                      Vec3& hit_point)
{
    const Vec3 origin{5, 2, 0};

    double closest_t = std::numeric_limits<double>::max();
    bool hit = false;

    for (const auto& tri : triangles_) {
        double t;
        if (intersectTriangle(origin, ray_dir, tri, t)) {
            if (t < closest_t) {
                closest_t = t;
                hit = true;
            }
        }
    }

    if (hit) {
        hit_point.x = ray_dir.x * closest_t;
        hit_point.y = ray_dir.y * closest_t;
        hit_point.z = ray_dir.z * closest_t;
    }

    return hit;
}

bool EmulationProducer::intersectTriangle(const Vec3& orig,
                                          const Vec3& dir,
                                          const Triangle& tri,
                                          double& t)
{
    const double EPS = 1e-8;

    Vec3 edge1{tri.v1.x - tri.v0.x,
               tri.v1.y - tri.v0.y,
               tri.v1.z - tri.v0.z};

    Vec3 edge2{tri.v2.x - tri.v0.x,
               tri.v2.y - tri.v0.y,
               tri.v2.z - tri.v0.z};

    Vec3 h{
        dir.y * edge2.z - dir.z * edge2.y,
        dir.z * edge2.x - dir.x * edge2.z,
        dir.x * edge2.y - dir.y * edge2.x
    };

    double a = edge1.x*h.x + edge1.y*h.y + edge1.z*h.z;
    if (fabs(a) < EPS) return false;

    double f = 1.0 / a;

    Vec3 s{orig.x - tri.v0.x,
           orig.y - tri.v0.y,
           orig.z - tri.v0.z};

    double u = f * (s.x*h.x + s.y*h.y + s.z*h.z);
    if (u < 0.0 || u > 1.0) return false;

    Vec3 q{
        s.y*edge1.z - s.z*edge1.y,
        s.z*edge1.x - s.x*edge1.z,
        s.x*edge1.y - s.y*edge1.x
    };

    double v = f * (dir.x*q.x + dir.y*q.y + dir.z*q.z);
    if (v < 0.0 || u + v > 1.0) return false;

    t = f * (edge2.x*q.x + edge2.y*q.y + edge2.z*q.z);
    return t > EPS;
}

EmulationProducer::~EmulationProducer() {
    running_ = false;
    if (production_thread_.joinable()) {
        production_thread_.join();
    }
}


void EmulationProducer::produce()
{
    const double vertical_min_deg = -90.0;
    const double vertical_max_deg =  90.0;

    while (running_)
    {
        unilidar_sdk2::PointCloudUnitree cloud;

        cloud.stamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        cloud.id = sequence_id_;
        cloud.ringNum = num_rings_;
        cloud.points.clear();
        cloud.points.reserve(num_rings_ * points_per_ring_);

        for (uint32_t ring = 0; ring < num_rings_; ++ring)
        {
            double vert_deg;

            if (num_rings_ > 1)
            {
                vert_deg = vertical_min_deg +
                    ring * (vertical_max_deg - vertical_min_deg) /
                    (num_rings_ - 1);
            }
            else
            {
                vert_deg = 0.0;
            }

            double vert_rad = vert_deg * M_PI / 180.0;

            for (uint32_t pt = 0; pt < points_per_ring_; ++pt)
            {
                double horiz_rad =
                    2.0 * M_PI * static_cast<double>(pt) /
                    static_cast<double>(points_per_ring_);

                auto point = generatePoint(horiz_rad, vert_rad);
                point.ring = ring;

                cloud.points.push_back(point);
            }
        }

        // Push to queue (thread safe)
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            cloud_queue_.push(std::move(cloud));
        }

        queue_cv_.notify_one();
        sequence_id_++;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

