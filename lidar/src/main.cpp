#include <thread>
#include <chrono>
#include <memory>
#include <cstring>
#include <string>

#include "udp_producer.h"
#include "npy_producer.h"
#include "config.h"
#include "emulation_producer.h"
#include "consumer.h"
#include "logger.h"

int main(int argc, char* argv[]) {

    if (argc < 3 || (strcmp(argv[1], "--config") != 0 && strcmp(argv[1], "-c") != 0)) {
        LOG_ERROR("Usage: %s --config <config.yaml>", argv[0]);
        return 1;
    }

    std::string config_file = argv[2];

    parkme_cfg_t cfg;

    try {
        cfg = parse_config(config_file);
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to parse config: %s", e.what());
        return 1;
    }

    LOG_INFO("Mode: %s", cfg.mode.c_str());

    std::shared_ptr<Producer> producer;

    if (cfg.mode == "emulation") {
	LOG_INFO("Rings: %u, Points/Ring: %u, Interval: %.3f s",
		 cfg.nof_rings, cfg.points_per_ring, cfg.cloud_generation_interval);
	LOG_INFO("WebSocket Port: %d", cfg.websocket_port);
	LOG_INFO("LiDAR offset: (%.2f, %.2f, %.2f)", cfg.lidar_x, cfg.lidar_y, cfg.lidar_z);

	producer = std::make_shared<EmulationProducer>(
	    cfg.nof_rings,
	    cfg.points_per_ring,
	    cfg.lidar_x,
	    cfg.lidar_y,
	    cfg.lidar_z,
	    cfg.scene_duration
	);
    } else if (cfg.mode == "udp") {
	LOG_INFO("UDP source %s:%d", cfg.udp_ip.c_str(), cfg.udp_port);
	LOG_INFO("WebSocket Port: %d", cfg.websocket_port);

	producer = std::make_shared<UdpProducer>(
	    cfg.lidar_ip,
	    static_cast<unsigned short>(cfg.lidar_port),
	    cfg.local_ip,
	    static_cast<unsigned short>(cfg.local_port)
	);
    } else if (cfg.mode == "replay") {
	LOG_INFO("Replay file: %s", cfg.npy_file.c_str());
	LOG_INFO("tilt=%.1f° pan=%.1f° roll=%.1f°  r=[%.1f,%.1f]  ground>%.2f  period=%.3fs",
		 cfg.tilt_deg, cfg.pan_deg, cfg.roll_deg,
		 cfg.min_radius, cfg.max_radius, cfg.ground_clearance, cfg.send_period_sec);
	LOG_INFO("WebSocket Port: %d", cfg.websocket_port);

	producer = std::make_shared<NpyProducer>(
	    cfg.npy_file,
	    cfg.tilt_deg,
	    cfg.pan_deg,
	    cfg.roll_deg,
	    cfg.min_radius,
	    cfg.max_radius,
	    cfg.ground_clearance,
	    cfg.send_period_sec
	);
    } else {
	LOG_ERROR("Unknown mode: %s", cfg.mode.c_str());
	return 1;
    }

    // Consumer runs its own worker thread internally
    ConsumerConfig consumer_cfg;
    consumer_cfg.ransac_distance_threshold = cfg.ransac_distance_threshold;
    consumer_cfg.ransac_max_iterations     = cfg.ransac_max_iterations;
    consumer_cfg.ground_above_plane        = cfg.ground_above_plane;
    consumer_cfg.cluster_tolerance         = cfg.cluster_tolerance;
    consumer_cfg.cluster_min_size          = cfg.cluster_min_size;
    consumer_cfg.cluster_max_size          = cfg.cluster_max_size;

    LOG_INFO("Ground removal: ransac_thresh=%.2fm  iters=%d  clearance=%.2fm",
             consumer_cfg.ransac_distance_threshold,
             consumer_cfg.ransac_max_iterations,
             consumer_cfg.ground_above_plane);
    LOG_INFO("Clustering: tolerance=%.2fm  min=%d  max=%d",
             consumer_cfg.cluster_tolerance,
             consumer_cfg.cluster_min_size,
             consumer_cfg.cluster_max_size);

    Consumer consumer(producer, cfg.websocket_port, consumer_cfg);

    LOG_INFO("Streaming point clouds to WebSocket...");

    // Keep application alive
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

