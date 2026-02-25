#ifndef EMULATION_PRODUCER_H
#define EMULATION_PRODUCER_H

#include "producer.h"
#include <cmath>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>
#include <numeric>
#include <functional>
#include <limits>

class EmulationProducer : public Producer {
public:
    EmulationProducer(const std::string& obj_path,
                      uint32_t num_rings = 16,
                      uint32_t points_per_ring = 100);
    ~EmulationProducer() override;

protected:
    void produce() override;

private:
    struct Vec3 {
        double x, y, z;
    };

    struct Triangle {
        Vec3 v0, v1, v2;
    };

    uint32_t num_rings_;
    uint32_t points_per_ring_;
    uint32_t sequence_id_;

    std::vector<Vec3> vertices_;

    struct AABB {
	Vec3 min;
	Vec3 max;
    };

    struct BVHNode {
	AABB box;
	int left = -1;
	int right = -1;
	std::vector<int> triangle_indices;
    };

    std::vector<Triangle> base_triangles_;
    std::vector<Triangle> triangles_;
    std::vector<BVHNode> bvh_;


    bool loadOBJ(const std::string& path);
    bool intersectMesh(const Vec3& origin, const Vec3& ray_dir, Vec3& hit_point);
    bool intersectTriangle(const Vec3& orig,
                           const Vec3& dir,
                           const Triangle& tri,
                           double& t);

    unilidar_sdk2::PointUnitree generatePoint(double angle_horizontal,
                                              double angle_vertical);

    void buildBVH();
    bool intersectAABB(const Vec3& orig, const Vec3& dir, const AABB& box);

    AABB computeAABB(const std::vector<int>& indices);
    int buildBVHRecursive(std::vector<int>& indices);
};

#endif

