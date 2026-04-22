#include "consumer.h"
#include "producer.h"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cmath>
#include <cstdio>
#include <iostream>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

Consumer::Consumer(std::shared_ptr<Producer> producer, uint16_t port, ConsumerConfig cfg)
    : producer_(producer),
      port_(port),
      cfg_(cfg),
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

    // --- 2. Ground removal then Euclidean clustering.
    //
    // Two-stage strategy:
    //
    // Stage A — RANSAC plane fit with a tight inlier threshold to estimate the
    //   ground plane equation precisely without absorbing car-body points.
    //
    // Stage B — Per-point signed-height sweep using the fitted plane equation.
    //   Every point whose signed height above the plane is less than
    //   cfg_.ground_above_plane (default 0.25 m) is marked ground and dropped.
    //   This is strictly more aggressive than the RANSAC inlier mask alone:
    //   residual pavement fragments up to 25 cm above the fitted surface are
    //   removed, eliminating the clusters of ground points that previously
    //   merged with the bottom of vehicle clusters.

    std::vector<int>  point_cluster_ids(hits.size(), -1);
    std::vector<bool> is_ground(hits.size(), false);

    if (!hits.empty()) {
        // Build a PCL cloud that indexes back into hits[].
        pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        full_cloud->points.reserve(hits.size());
        for (const auto* p : hits)
            full_cloud->points.emplace_back(p->x, p->y, p->z);
        full_cloud->width  = (uint32_t)full_cloud->points.size();
        full_cloud->height = 1;
        full_cloud->is_dense = true;

        // Stage A: RANSAC plane fit.
        // distanceThreshold is kept tight so the fitted plane is accurate —
        // the generous clearance is applied in Stage B, not here.
        pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr      plane_inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(cfg_.ransac_distance_threshold);
        seg.setMaxIterations(cfg_.ransac_max_iterations);
        seg.setInputCloud(full_cloud);
        seg.segment(*plane_inliers, *plane_coeff);

        // Stage B: signed-height sweep.
        // Plane equation: a*x + b*y + c*z + d = 0  →  normal n = (a,b,c).
        // Signed distance of point P above the plane:
        //   h = (a*Px + b*Py + c*Pz + d) / ||n||
        // Positive h → point is in the direction the normal points.
        // We enforce c > 0 so the normal always points toward +z (upward),
        // making h > 0 mean "above the ground" in the scene's vertical axis.
        if (!plane_inliers->indices.empty() && plane_coeff->values.size() == 4) {
            float a = plane_coeff->values[0];
            float b = plane_coeff->values[1];
            float c = plane_coeff->values[2];
            float d = plane_coeff->values[3];
            float mag = std::sqrt(a*a + b*b + c*c);

            // Accept only near-horizontal planes (|n_z| / ||n|| > 0.7, i.e. up to ~45° tilt).
            // This rejects walls and other vertical surfaces the RANSAC may
            // mistake for the dominant plane when few ground points are visible.
            if (mag > 1e-6f && std::abs(c / mag) > 0.7f) {
                // Normalise so h is in metres and the sign means "above ground".
                if (c < 0.0f) { a = -a; b = -b; c = -c; d = -d; }
                float inv_mag = 1.0f / mag;

                for (int i = 0; i < (int)hits.size(); ++i) {
                    float h = (a * hits[i]->x + b * hits[i]->y
                               + c * hits[i]->z + d) * inv_mag;
                    // Below the plane (h < 0) → underground noise, always drop.
                    // Within clearance above the plane → pavement / near-ground, drop.
                    if (h < cfg_.ground_above_plane)
                        is_ground[i] = true;
                }
            }
        }

        // Build the clustering cloud from non-ground hits only.
        // pcl_to_hits[i] maps PCL index → original hits[] index so cluster IDs
        // can be written back correctly.
        std::vector<int> pcl_to_hits;
        pcl_to_hits.reserve(hits.size());
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_cloud->points.reserve(hits.size());
        for (int i = 0; i < (int)hits.size(); ++i) {
            if (!is_ground[i]) {
                pcl_cloud->points.emplace_back(hits[i]->x, hits[i]->y, hits[i]->z);
                pcl_to_hits.push_back(i);
            }
        }
        pcl_cloud->width  = (uint32_t)pcl_cloud->points.size();
        pcl_cloud->height = 1;
        pcl_cloud->is_dense = true;

        if (!pcl_cloud->points.empty()) {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(pcl_cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(cfg_.cluster_tolerance);
            ec.setMinClusterSize(cfg_.cluster_min_size);
            ec.setMaxClusterSize(cfg_.cluster_max_size);
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
    //        Ground points are omitted entirely so the renderer never sees them.
    std::string out;
    out.reserve(128 + hits.size() * 80);

    char hdr[128];
    std::snprintf(hdr, sizeof(hdr),
                  "{\"stamp\":%.6f,\"id\":%u,\"ringNum\":%u,\"points\":[",
                  cloud.stamp, cloud.id, cloud.ringNum);
    out += hdr;

    char buf[128];
    bool first = true;
    for (size_t i = 0; i < hits.size(); ++i) {
        if (is_ground[i]) continue;
        if (!first) out += ',';
        first = false;
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
