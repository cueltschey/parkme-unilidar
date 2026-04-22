#include "udp_producer.h"
#include "logger.h"

// Use the actual SDK header path from your project
#include "example.h"

UdpProducer::UdpProducer(const std::string& lidar_ip,
                         unsigned short lidar_port,
                         const std::string& local_ip,
                         unsigned short local_port)
    : lidar_ip_(lidar_ip),
      local_ip_(local_ip),
      lidar_port_(lidar_port),
      local_port_(local_port)
{
    running_ = true;
    production_thread_ = std::thread(&UdpProducer::produce, this);
}

UdpProducer::~UdpProducer() {
    running_ = false;
    queue_cv_.notify_all();
    if (production_thread_.joinable()) {
        production_thread_.join();
    }
}

void UdpProducer::produce() {
    UnitreeLidarReader *lreader = createUnitreeLidarReader();
    if (!lreader) {
        LOG_ERROR("Failed to create UnitreeLidarReader");
        running_ = false;
        return;
    }

    if (lreader->initializeUDP(lidar_port_, lidar_ip_, local_port_, local_ip_)) {
        LOG_ERROR("Unilidar initialization failed");
        running_ = false;
        return;
    }

    LOG_INFO("Unilidar initialization succeeded");

    lreader->startLidarRotation();
    sleep(1);

    uint32_t workMode = 0;
    lreader->setLidarWorkMode(workMode);
    sleep(1);

    lreader->resetLidar();
    sleep(1);

    while (running_) {
        unilidar_sdk2::PointCloudUnitree cloud;

        // Replace this with the SDK call your exampleProcess uses
        // For example, if the SDK exposes a method that fills cloud directly:
        // bool ok = lreader->getCloudFrame(cloud);

        bool ok = false; // TODO: replace with real SDK frame-read call

        if (!ok) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            cloud_queue_.push(std::move(cloud));
        }
        queue_cv_.notify_one();
    }

    delete lreader;
}
