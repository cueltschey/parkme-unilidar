#include "consumer.h"
#include "producer.h"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <iostream>

using json = nlohmann::json;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

Consumer::Consumer(std::shared_ptr<Producer> producer, uint16_t port)
    : producer_(producer),
      port_(port),
      acceptor_(ioc_, tcp::endpoint(tcp::v4(), port))
{
    worker_thread_ = std::thread(&Consumer::run, this);
}

Consumer::~Consumer() {
    running_ = false;
    ioc_.stop();

    if (worker_thread_.joinable())
        worker_thread_.join();
}

void Consumer::run() {
    acceptConnections();

    // Main streaming loop
    while (running_) {
        auto cloud = producer_->getCloud();
        json j = cloudToJson(cloud);

        broadcastCloud(j);

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // adjust for frame rate
    }
}

void Consumer::acceptConnections() {
    std::thread([this]() {
        while (running_) {
            try {
                tcp::socket socket(ioc_);
                acceptor_.accept(socket);

                auto ws = std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
                ws->accept();

                {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    clients_.push_back(ws);
                }

                std::cout << "New WebSocket client connected" << std::endl;

            } catch (const std::exception& e) {
                if (running_)
                    std::cerr << "Accept error: " << e.what() << std::endl;
            }
        }
    }).detach();
}

void Consumer::broadcastCloud(const json& j) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    auto it = clients_.begin();
    while (it != clients_.end()) {
        try {
            (*it)->write(net::buffer(j.dump()));
            ++it;
        } catch (const std::exception& e) {
            std::cerr << "WebSocket send error: " << e.what() << ", removing client" << std::endl;
            it = clients_.erase(it);
        }
    }
}

json Consumer::cloudToJson(const unilidar_sdk2::PointCloudUnitree& cloud) {
    json j;
    j["stamp"] = cloud.stamp;
    j["id"] = cloud.id;
    j["ringNum"] = cloud.ringNum;

    j["points"] = json::array();
    for (const auto& p : cloud.points) {
        j["points"].push_back({
            {"x", p.x},
            {"y", p.y},
            {"z", p.z},
            {"intensity", p.intensity},
            {"ring", p.ring},
            {"time", p.time}
        });
    }

    return j;
}

