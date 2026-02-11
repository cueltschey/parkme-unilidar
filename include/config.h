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
    std::string mode; // "emulation" or "udp"
    std::string udp_ip;
    uint16_t udp_port = 0;
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
    }
    else if (node["testbed"]) {
        YAML::Node tb = node["testbed"];
        config.mode = "udp";
        if (tb["udp_ip"])
            config.udp_ip = tb["udp_ip"].as<std::string>();
        if (tb["udp_port"])
            config.udp_port = tb["udp_port"].as<uint16_t>();
    } else {
        std::cerr << "Error: YAML must contain either 'emulation' or 'testbed' section." << std::endl;
        throw std::runtime_error("Invalid YAML configuration");
    }

    return config;
}

