#pragma once

#include "producer.h"
#include <string>
#include <vector>

class NpyProducer : public Producer {
public:
    NpyProducer(const std::string& npy_file,
                float tilt_deg = 0.0f,
                float pan_deg  = 0.0f,
                float roll_deg = 0.0f,
                float min_radius = 4.0f,
                float max_radius = 80.0f,
                float ground_clearance = 0.0f,
                double send_period_sec = 0.2);
    ~NpyProducer() override;

protected:
    void produce() override;

private:
    // Filtered, tilt-corrected, z-adjusted points stored flat as x,y,z triplets.
    std::vector<float> points_;
    double send_period_sec_;

    void loadNpy(const std::string& path,
                 float tilt_deg,
                 float pan_deg,
                 float roll_deg,
                 float min_radius,
                 float max_radius,
                 float ground_clearance);
};
