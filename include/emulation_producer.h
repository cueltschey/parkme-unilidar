#ifndef EMULATION_PRODUCER_H
#define EMULATION_PRODUCER_H

#include "producer.h"
#include <cmath>

class EmulationProducer : public Producer {
public:
    EmulationProducer(uint32_t num_rings = 16, uint32_t points_per_ring = 100);
    ~EmulationProducer() override;

protected:
    void produce() override;

private:
    uint32_t num_rings_;
    uint32_t points_per_ring_;
    uint32_t sequence_id_;

    unilidar_sdk2::PointUnitree generatePoint(double angle_horizontal, double angle_vertical);
    bool intersectSphere(double x, double y, double z, double& hit_x, double& hit_y, double& hit_z);
};

#endif // EMULATION_PRODUCER_H
