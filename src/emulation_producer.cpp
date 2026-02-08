#include "emulation_producer.h"
#include <iostream>

EmulationProducer::EmulationProducer(uint32_t num_rings, uint32_t points_per_ring)
    : num_rings_(num_rings), points_per_ring_(points_per_ring), sequence_id_(0) 
{
    production_thread_ = std::thread(&EmulationProducer::produce, this);
}

EmulationProducer::~EmulationProducer() {
    running_ = false;
    if (production_thread_.joinable()) {
        production_thread_.join();
    }
}

void EmulationProducer::produce() {
    while (running_) {
        unilidar_sdk2::PointCloudUnitree cloud;
        cloud.stamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        cloud.id = sequence_id_;
        cloud.ringNum = num_rings_;

        for (uint32_t ring = 0; ring < num_rings_; ++ring) {
            double vert_angle = -15.0 + ring * (30.0 / std::max(1u, num_rings_-1)); // example
            for (uint32_t pt = 0; pt < points_per_ring_; ++pt) {
                double horiz_angle = 2 * M_PI * pt / points_per_ring_;
                unilidar_sdk2::PointUnitree p = generatePoint(horiz_angle, vert_angle);
                p.ring = ring;
                cloud.points.push_back(p);
            }
        }

        // Push to queue
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            cloud_queue_.push(cloud);
        }
        queue_cv_.notify_one();
        sequence_id_++;

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // simulate lidar rate
    }
}

unilidar_sdk2::PointUnitree EmulationProducer::generatePoint(double angle_horizontal, double angle_vertical) {
    unilidar_sdk2::PointUnitree point{};
    double distance = 5.0; // default distance to ground if nothing else

    double x = distance * cos(angle_vertical) * cos(angle_horizontal);
    double y = distance * cos(angle_vertical) * sin(angle_horizontal);
    double z = distance * sin(angle_vertical);

    // Check intersection with a simple sphere at (0,0,1) radius 1
    double hit_x, hit_y, hit_z;
    if (intersectSphere(x, y, z, hit_x, hit_y, hit_z)) {
        point.x = hit_x;
        point.y = hit_y;
        point.z = hit_z;
        point.intensity = 1.0f;
    } else {
        point.x = x;
        point.y = y;
        point.z = 0.0; // ground plane at z=0
        point.intensity = 0.5f;
    }

    point.time = 0.0; // for simplicity, can simulate per-point timing
    return point;
}

bool EmulationProducer::intersectSphere(double x, double y, double z, double& hit_x, double& hit_y, double& hit_z) {
    // Sphere at (0,0,1) radius 1
    double cx = 0.0, cy = 0.0, cz = 1.0, r = 1.0;
    double dx = x, dy = y, dz = z - 1.0; // ray from origin
    double a = dx*dx + dy*dy + dz*dz;
    double b = 2*(dx*(-cx) + dy*(-cy) + dz*(-cz));
    double c = cx*cx + cy*cy + cz*cz - r*r;
    double disc = b*b - 4*a*c;
    if (disc < 0) return false;

    double t = (-b - sqrt(disc)) / (2*a);
    if (t < 0) return false;

    hit_x = t*dx;
    hit_y = t*dy;
    hit_z = t*dz + 0.0;
    return true;
}

