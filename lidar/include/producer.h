#ifndef PRODUCER_H
#define PRODUCER_H

#include "unitree_lidar_utilities.h"
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

class Producer {
public:
    Producer();
    virtual ~Producer();

    unilidar_sdk2::PointCloudUnitree getCloud();

protected:
    virtual void produce() = 0;

    std::thread production_thread_;
    std::queue<unilidar_sdk2::PointCloudUnitree> cloud_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> running_;
};

#endif // PRODUCER_H
