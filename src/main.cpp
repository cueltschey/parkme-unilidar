#include <thread>
#include <chrono>
#include <memory>
#include <cstring>
#include <string>

#include "config.h"
#include "emulation_producer.h"
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
                 cfg.nof_rings, cfg.points_per_ring, cfg.cloud_generation_interval);
        LOG_INFO("WebSocket URL: %s", cfg.websocket_url.c_str());
    } else if (cfg.mode == "udp") {
        LOG_INFO("UDP IP: %s, Port: %u", cfg.udp_ip.c_str(), cfg.udp_port);
    }

    std::unique_ptr<Producer> producer;

    if (cfg.mode == "emulation") {
        producer = std::make_unique<EmulationProducer>(cfg.nof_rings, cfg.points_per_ring);
    } else if (cfg.mode == "udp") {
        LOG_ERROR("UDP mode not implemented yet");
        return 1;
    } else {
        LOG_ERROR("Unknown mode: %s", cfg.mode.c_str());
        return 1;
    }

    LOG_INFO("Starting point cloud loop...");

    while (true) {
        auto cloud = producer->getCloud();
        LOG_DEBUG("Received cloud id: %u, points: %zu", cloud.id, cloud.points.size());
        std::this_thread::sleep_for(std::chrono::duration<double>(cfg.cloud_generation_interval));
    }

    return 0;
}

