#include "producer.h"
#include <iostream>
#include <chrono>

Producer::Producer() : running_(true) {
    // Derived class should start the thread
}

Producer::~Producer() {
    running_ = false;
    if (production_thread_.joinable()) {
        production_thread_.join();
    }
}

unilidar_sdk2::PointCloudUnitree Producer::getCloud() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait(lock, [this]() { return !cloud_queue_.empty() || !running_; });

    if (cloud_queue_.empty()) {
        return unilidar_sdk2::PointCloudUnitree{}; // return empty if stopped
    }

    unilidar_sdk2::PointCloudUnitree cloud = cloud_queue_.front();
    cloud_queue_.pop();
    return cloud;
}
