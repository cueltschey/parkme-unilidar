#ifndef CONSUMER_H
#define CONSUMER_H

#include "unitree_lidar_utilities.h"
#include <nlohmann/json.hpp>

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

class Consumer : public std::enable_shared_from_this<Consumer> {
public:
    Consumer(std::shared_ptr<Producer> producer,
             uint16_t port);

    ~Consumer();

private:
    void run();
    void acceptConnections();
    void broadcastCloud(const nlohmann::json& j);
    nlohmann::json cloudToJson(
        const unilidar_sdk2::PointCloudUnitree& cloud);

    std::shared_ptr<Producer> producer_;
    uint16_t port_;

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

