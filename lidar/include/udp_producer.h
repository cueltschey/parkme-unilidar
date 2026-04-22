#pragma once

#include "producer.h"
#include <string>

class UdpProducer : public Producer {
public:
    UdpProducer(const std::string& lidar_ip,
                unsigned short lidar_port,
                const std::string& local_ip,
                unsigned short local_port);
    ~UdpProducer() override;

protected:
    void produce() override;

private:
    std::string lidar_ip_;
    std::string local_ip_;
    unsigned short lidar_port_;
    unsigned short local_port_;
};
