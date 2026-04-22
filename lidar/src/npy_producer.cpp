#include "npy_producer.h"
#include "logger.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <cstring>
#include <cmath>
#include <chrono>

// ---------------------------------------------------------------------------
// Minimal .npy v1.x / v2.x parser (float32, C-order, shape (N,3) only).
// ---------------------------------------------------------------------------
static size_t parseNpyHeader(std::ifstream& f) {
    // Magic + version
    char magic[6];
    f.read(magic, 6);
    if (std::memcmp(magic, "\x93NUMPY", 6) != 0)
        throw std::runtime_error("Not a .npy file");

    uint8_t major, minor;
    f.read(reinterpret_cast<char*>(&major), 1);
    f.read(reinterpret_cast<char*>(&minor), 1);

    // Header length: 2 bytes for v1.x, 4 bytes for v2.x
    uint32_t header_len = 0;
    if (major == 1) {
        uint16_t hl16;
        f.read(reinterpret_cast<char*>(&hl16), 2);
        header_len = hl16;
    } else {
        f.read(reinterpret_cast<char*>(&header_len), 4);
    }

    std::string header(header_len, '\0');
    f.read(&header[0], header_len);

    // Extract the first dimension from 'shape': (N, 3)
    auto shapePos = header.find("'shape'");
    if (shapePos == std::string::npos)
        shapePos = header.find("\"shape\"");
    if (shapePos == std::string::npos)
        throw std::runtime_error(".npy header missing 'shape'");

    auto paren = header.find('(', shapePos);
    if (paren == std::string::npos)
        throw std::runtime_error(".npy header: malformed shape");

    size_t n = std::stoull(header.substr(paren + 1));
    return n;
}

void NpyProducer::loadNpy(const std::string& path,
                           float tilt_deg,
                           float pan_deg,
                           float roll_deg,
                           float min_radius,
                           float max_radius,
                           float ground_clearance)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("Cannot open npy file: " + path);

    size_t n = parseNpyHeader(f);
    LOG_INFO("NpyProducer: reading %zu points from %s", n, path.c_str());

    std::vector<float> raw(n * 3);
    f.read(reinterpret_cast<char*>(raw.data()), n * 3 * sizeof(float));
    if (!f)
        throw std::runtime_error("Truncated .npy data in: " + path);

    // Build ZYX rotation matrix: R = Rz(pan) * Ry(tilt) * Rx(roll)
    const float deg2rad = (float)M_PI / 180.0f;
    float cr = std::cos(roll_deg  * deg2rad), sr = std::sin(roll_deg  * deg2rad);
    float ct = std::cos(tilt_deg  * deg2rad), st = std::sin(tilt_deg  * deg2rad);
    float cp = std::cos(pan_deg   * deg2rad), sp = std::sin(pan_deg   * deg2rad);

    // Row-major entries of R = Rz*Ry*Rx
    float r00 = cp*ct,  r01 = cp*st*sr - sp*cr,  r02 = cp*st*cr + sp*sr;
    float r10 = sp*ct,  r11 = sp*st*sr + cp*cr,  r12 = sp*st*cr - cp*sr;
    float r20 = -st,    r21 = ct*sr,              r22 = ct*cr;

    // Two-pass: filter by radius, apply rotation, collect z for percentile.
    std::vector<float> filtered;
    filtered.reserve(n * 3);
    std::vector<float> zvals;
    zvals.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        float x = raw[i*3 + 0];
        float y = raw[i*3 + 1];
        float z = raw[i*3 + 2];

        float r = std::sqrt(x*x + y*y + z*z);
        if (!std::isfinite(r) || r <= min_radius || r >= max_radius)
            continue;

        float rx = r00*x + r01*y + r02*z;
        float ry = r10*x + r11*y + r12*z;
        float rz = r20*x + r21*y + r22*z;

        filtered.push_back(rx);
        filtered.push_back(ry);
        filtered.push_back(rz);
        zvals.push_back(rz);
    }

    if (zvals.empty())
        throw std::runtime_error("No points remain after radius filter in: " + path);

    // Shift z so the 2nd-percentile sits at 0.
    std::sort(zvals.begin(), zvals.end());
    float z0 = zvals[static_cast<size_t>(0.02f * zvals.size())];

    size_t count = filtered.size() / 3;
    for (size_t i = 0; i < count; ++i)
        filtered[i*3 + 2] -= z0;

    // Remove ground plane: keep only points above ground_clearance.
    std::vector<float> above_ground;
    above_ground.reserve(filtered.size());
    for (size_t i = 0; i < count; ++i) {
        if (filtered[i*3 + 2] > ground_clearance) {
            above_ground.push_back(filtered[i*3 + 0]);
            above_ground.push_back(filtered[i*3 + 1]);
            above_ground.push_back(filtered[i*3 + 2]);
        }
    }

    points_ = std::move(above_ground);
    LOG_INFO("NpyProducer: %zu points after filter (tilt=%.1f° pan=%.1f° roll=%.1f°, r=[%.1f,%.1f], ground>%.2f)",
             points_.size() / 3, tilt_deg, pan_deg, roll_deg, min_radius, max_radius, ground_clearance);
}

NpyProducer::NpyProducer(const std::string& npy_file,
                         float tilt_deg,
                         float pan_deg,
                         float roll_deg,
                         float min_radius,
                         float max_radius,
                         float ground_clearance,
                         double send_period_sec)
    : send_period_sec_(send_period_sec)
{
    loadNpy(npy_file, tilt_deg, pan_deg, roll_deg, min_radius, max_radius, ground_clearance);
    running_ = true;
    production_thread_ = std::thread(&NpyProducer::produce, this);
}

NpyProducer::~NpyProducer() {
    running_ = false;
    queue_cv_.notify_all();
    if (production_thread_.joinable())
        production_thread_.join();
}

void NpyProducer::produce() {
    uint32_t seq = 0;
    const size_t count = points_.size() / 3;

    while (running_) {
        unilidar_sdk2::PointCloudUnitree cloud;
        cloud.stamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        cloud.id      = seq;
        cloud.ringNum = 1;
        cloud.points.resize(count);

        for (size_t i = 0; i < count; ++i) {
            auto& p    = cloud.points[i];
            p.x         = points_[i*3 + 0];
            p.y         = points_[i*3 + 1];
            p.z         = points_[i*3 + 2];
            p.intensity = 1.0f;
            p.time      = 0.0f;
            p.ring      = 0;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            cloud_queue_.push(std::move(cloud));
        }
        queue_cv_.notify_one();
        ++seq;

        std::this_thread::sleep_for(
            std::chrono::duration<double>(send_period_sec_));
    }
}
