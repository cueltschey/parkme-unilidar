#ifndef CONSUMER_H
#define CONSUMER_H

#include "unitree_lidar_utilities.h"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>

#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

class Producer;

// All tunable parameters for ground removal and Euclidean clustering.
// Populated from the YAML config and forwarded to the Consumer at construction.
struct ConsumerConfig {
    // RANSAC ground plane fit
    float ransac_distance_threshold = 0.10f;  // metres — tight fit for plane estimation
    int   ransac_max_iterations     = 100;
    // How far above the fitted plane a point can be and still be considered ground.
    // Raising this removes more near-ground clutter at the cost of clipping low car parts.
    float ground_above_plane        = 0.25f;  // metres

    // Euclidean cluster extraction
    float cluster_tolerance         = 0.40f;  // metres — max distance between cluster neighbours
    int   cluster_min_size          = 10;
    int   cluster_max_size          = 25000;
};

class Consumer : public std::enable_shared_from_this<Consumer> {
public:
    Consumer(std::shared_ptr<Producer> producer,
             uint16_t port,
             ConsumerConfig cfg = {});

    ~Consumer();

private:
    void run();
    void acceptConnections();
    void broadcastCloud(const std::string& payload);
    std::string cloudToJson(
        const unilidar_sdk2::PointCloudUnitree& cloud);

    std::shared_ptr<Producer> producer_;
    uint16_t port_;
    ConsumerConfig cfg_;

    std::thread worker_thread_;
    std::atomic<bool> running_{true};

    // Boost.Asio / Beast
    boost::asio::io_context ioc_;
    boost::asio::ip::tcp::acceptor acceptor_;

    // Keep track of connected clients
    std::vector<std::shared_ptr<boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>> clients_;
    std::mutex clients_mutex_;
};

#endif // CONSUMER_H

