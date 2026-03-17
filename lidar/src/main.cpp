#include <thread>
#include <chrono>
#include <memory>
#include <cstring>
#include <string>

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

    if (cfg.mode == "emulation") {
        LOG_INFO("Rings: %u, Points/Ring: %u, Interval: %.3f s",
                 cfg.nof_rings,
                 cfg.points_per_ring,
                 cfg.cloud_generation_interval);

        LOG_INFO("WebSocket Port: %d", cfg.websocket_port);
        LOG_INFO("LiDAR offset: (%.2f, %.2f, %.2f)", cfg.lidar_x, cfg.lidar_y, cfg.lidar_z);
    } 
    else if (cfg.mode == "udp") {
        LOG_ERROR("UDP mode not implemented yet");
        return 1;
    } 
    else {
        LOG_ERROR("Unknown mode: %s", cfg.mode.c_str());
        return 1;
    }

    auto producer = std::make_shared<EmulationProducer>(
        cfg.nof_rings,
        cfg.points_per_ring,
        cfg.lidar_x,
        cfg.lidar_y,
        cfg.lidar_z
    );

    // Consumer runs its own worker thread internally
    Consumer consumer(producer, cfg.websocket_port);

    LOG_INFO("Streaming point clouds to WebSocket...");

    // Keep application alive
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

