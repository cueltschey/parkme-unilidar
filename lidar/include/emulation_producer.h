#ifndef EMULATION_PRODUCER_H
#define EMULATION_PRODUCER_H

#include "producer.h"
#include <cmath>
#include <memory>

// Forward-declared so lidarsim.h (which has function bodies) is only included
// from emulation_producer.cpp, avoiding multiple-definition linker errors.
struct EmulationScene;

class EmulationProducer : public Producer {
public:
    // num_rings / points_per_ring accepted for config compatibility but unused;
    // ray pattern is loaded from csv/mid360.csv.
    EmulationProducer(uint32_t num_rings = 16,
                      uint32_t points_per_ring = 100,
                      float lidar_x = 0.0f,
                      float lidar_y = 0.0f,
                      float lidar_z = 1.0f);
    ~EmulationProducer() override;

protected:
    void produce() override;

private:
    uint32_t sequence_id_;
    float lidar_x_, lidar_y_, lidar_z_;

    std::unique_ptr<EmulationScene> scene_;

    void initScene();
};

#endif
