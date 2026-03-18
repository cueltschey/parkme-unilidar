#include "consumer.h"
#include "producer.h"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cstdio>
#include <iostream>

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

    while (running_) {
        auto cloud = producer_->getCloud();
        std::string payload = cloudToJson(cloud);
        broadcastCloud(payload);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

// Serialize once, write the same buffer to every client.
void Consumer::broadcastCloud(const std::string& payload) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    auto it = clients_.begin();
    while (it != clients_.end()) {
        try {
            (*it)->write(net::buffer(payload));
            ++it;
        } catch (const std::exception& e) {
            std::cerr << "WebSocket send error: " << e.what() << ", removing client" << std::endl;
            it = clients_.erase(it);
        }
    }
}

std::string Consumer::cloudToJson(const unilidar_sdk2::PointCloudUnitree& cloud) {
    // --- 1. Collect hit points only (intensity > 0 means the ray struck a mesh).
    //        Zero-intensity miss points are skipped entirely so PCL and the JSON
    //        serializer never touch the bulk of the 800 K rays that hit nothing.
    std::vector<const unilidar_sdk2::PointUnitree*> hits;
    hits.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        if (p.intensity > 0.0f)
            hits.push_back(&p);
    }

    // --- 2. PCL Euclidean clustering on the (small) hit set.
    std::vector<int> point_cluster_ids(hits.size(), -1);

    if (!hits.empty()) {
        // Build a filtered PCL cloud excluding near-ground hits.
        // The lidar sits at z ≈ +1 m in world space, so in lidar-relative coords
        // the ground plane is at z ≈ -1.0 m and car bodies start at z ≈ -0.87 m.
        // Points within the ground band (z < -0.9) are within PCL's 0.4 m
        // tolerance of wheel-level car points, causing them to merge into one
        // giant cluster.  Filtering them out before clustering isolates each car.
        //
        // pcl_to_hits[i] maps PCL cloud index → original hits[] index so that
        // cluster assignments can be written back correctly.
        std::vector<int> pcl_to_hits;
        pcl_to_hits.reserve(hits.size());

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_cloud->points.reserve(hits.size());
        for (int i = 0; i < (int)hits.size(); ++i) {
            if (hits[i]->z > -0.9f) {
                pcl_cloud->points.emplace_back(hits[i]->x, hits[i]->y, hits[i]->z);
                pcl_to_hits.push_back(i);
            }
        }
        pcl_cloud->width  = pcl_cloud->points.size();
        pcl_cloud->height = 1;
        pcl_cloud->is_dense = true;

        if (!pcl_cloud->points.empty()) {
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

            int cluster_id = 0;
            for (const auto& indices : cluster_indices) {
                for (int idx : indices.indices)
                    point_cluster_ids[pcl_to_hits[idx]] = cluster_id;
                ++cluster_id;
            }
        }
    }

    // --- 3. Build the JSON string directly with snprintf.
    //        Avoids the two-pass cost of building an nlohmann DOM tree and then
    //        calling dump() — each push_back on a large JSON array allocates heap
    //        nodes; snprintf into a pre-reserved string is a single linear pass.
    std::string out;
    out.reserve(128 + hits.size() * 80);

    char hdr[128];
    std::snprintf(hdr, sizeof(hdr),
                  "{\"stamp\":%.6f,\"id\":%u,\"ringNum\":%u,\"points\":[",
                  cloud.stamp, cloud.id, cloud.ringNum);
    out += hdr;

    char buf[128];
    for (size_t i = 0; i < hits.size(); ++i) {
        if (i > 0) out += ',';
        const auto* p = hits[i];
        std::snprintf(buf, sizeof(buf),
                      "{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,"
                      "\"intensity\":%.2f,\"ring\":%u,\"time\":%.4f,"
                      "\"cluster_id\":%d}",
                      p->x, p->y, p->z,
                      p->intensity, (unsigned)p->ring, p->time,
                      point_cluster_ids[i]);
        out += buf;
    }
    out += "]}";
    return out;
}
