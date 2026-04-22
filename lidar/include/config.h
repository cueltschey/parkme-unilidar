#include <yaml-cpp/yaml.h>
#include <cstring>
#include <string>
#include <iostream>

#include "logger.h"

typedef struct parkme_cfg_s {
    uint32_t nof_rings = 16;
    uint32_t points_per_ring = 100;
    uint64_t websocket_port;
    double cloud_generation_interval = 0.1; // seconds
    std::string mode; // "emulation", "udp", or "replay"
    std::string udp_ip;
    uint16_t udp_port = 0;
    std::string lidar_ip;
    uint16_t lidar_port = 0;
    std::string local_ip;
    uint16_t local_port = 0;
    std::string obj_file;
    float lidar_x = 0.0f;
    float lidar_y = 0.0f;
    float lidar_z = 1.0f; // default 1 m above ground
    double scene_duration = 10.0; // seconds between scene randomizations
    // replay mode
    std::string npy_file;
    float tilt_deg = 0.0f;
    float pan_deg  = 0.0f;
    float roll_deg = 0.0f;
    float min_radius = 4.0f;
    float max_radius = 80.0f;
    float ground_clearance = 0.0f;
    double send_period_sec = 0.2;
} parkme_cfg_t;

parkme_cfg_t parse_config(const std::string& config_file) {
    parkme_cfg_t config;

    YAML::Node node = YAML::LoadFile(config_file);

    if(node["log_level"]){
			std::string conf_log_level = node["log_level"].as<std::string>();
			if(conf_log_level == "DEBUG")
				log_level = DEBUG;
			else if(conf_log_level == "INFO")
				log_level = INFO;
			else if(conf_log_level == "WARNING")
				log_level = WARNING;
			else
				log_level = ERROR;
    }

    if (node["emulation"]) {
        YAML::Node emu = node["emulation"];
        config.mode = "emulation";
        if (emu["nof_rings"])
            config.nof_rings = emu["nof_rings"].as<uint32_t>();
        if (emu["points_per_ring"])
            config.points_per_ring = emu["points_per_ring"].as<uint32_t>();
        if (emu["websocket_port"])
            config.websocket_port = emu["websocket_port"].as<uint64_t>();
        if (emu["cloud_generation_interval"])
            config.cloud_generation_interval = emu["cloud_generation_interval"].as<double>();
        if (emu["obj_file"])
            config.obj_file = emu["obj_file"].as<std::string>();
        if (emu["lidar_x"])
            config.lidar_x = emu["lidar_x"].as<float>();
        if (emu["lidar_y"])
            config.lidar_y = emu["lidar_y"].as<float>();
        if (emu["lidar_z"])
            config.lidar_z = emu["lidar_z"].as<float>();
        if (emu["scene_duration"])
            config.scene_duration = emu["scene_duration"].as<double>();
    }
    else if (node["testbed"]) {
        YAML::Node tb = node["testbed"];
        config.mode = "udp";
        if (tb["udp_ip"])
            config.udp_ip = tb["udp_ip"].as<std::string>();
        if (tb["udp_port"])
            config.udp_port = tb["udp_port"].as<uint16_t>();
        if (tb["lidar_ip"])
            config.lidar_ip = tb["lidar_ip"].as<std::string>();
        if (tb["lidar_port"])
            config.lidar_port = tb["lidar_port"].as<uint16_t>();
        if (tb["local_ip"])
            config.local_ip = tb["local_ip"].as<std::string>();
        if (tb["local_port"])
            config.local_port = tb["local_port"].as<uint16_t>();
    } else if (node["replay"]) {
        YAML::Node rp = node["replay"];
        config.mode = "replay";
        if (rp["npy_file"])
            config.npy_file = rp["npy_file"].as<std::string>();
        if (rp["websocket_port"])
            config.websocket_port = rp["websocket_port"].as<uint64_t>();
        if (rp["tilt_deg"])
            config.tilt_deg = rp["tilt_deg"].as<float>();
        if (rp["pan_deg"])
            config.pan_deg = rp["pan_deg"].as<float>();
        if (rp["roll_deg"])
            config.roll_deg = rp["roll_deg"].as<float>();
        if (rp["min_radius"])
            config.min_radius = rp["min_radius"].as<float>();
        if (rp["max_radius"])
            config.max_radius = rp["max_radius"].as<float>();
        if (rp["ground_clearance"])
            config.ground_clearance = rp["ground_clearance"].as<float>();
        if (rp["send_period_sec"])
            config.send_period_sec = rp["send_period_sec"].as<double>();
    } else {
        std::cerr << "Error: YAML must contain 'emulation', 'testbed', or 'replay' section." << std::endl;
        throw std::runtime_error("Invalid YAML configuration");
    }

    return config;
}

