#include "consumer.h"
#include "producer.h"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud->points.reserve(cloud.points.size());

    for (const auto& p : cloud.points) {
        pcl_cloud->points.emplace_back(p.x, p.y, p.z);
    }

    pcl_cloud->width = pcl_cloud->points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pcl_cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.4);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pcl_cloud);
    ec.extract(cluster_indices);

    // -----------------------------
    // Map each point to a cluster ID
    // -----------------------------
    std::vector<int> point_cluster_ids(pcl_cloud->points.size(), -1);

    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
        for (int idx : indices.indices) {
            point_cluster_ids[idx] = cluster_id;
        }
        cluster_id++;
    }

    // -----------------------------
    // Build JSON output
    // -----------------------------
    j["points"] = json::array();

    for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];

        j["points"].push_back({
            {"x", p.x},
            {"y", p.y},
            {"z", p.z},
            {"intensity", p.intensity},
            {"ring", p.ring},
            {"time", p.time},
            {"cluster_id", point_cluster_ids[i]}   // <-- NEW FIELD
        });
    }

    return j;
}
