#pragma once

#include <cstddef>

struct LQRGainPoint {
    float airspeed_mps = 0.0f;

    float k_phi = 0.0f;
    float k_p = 0.0f;

    float k_theta = 0.0f;
    float k_q = 0.0f;
};

struct LQRScheduleTable {
    const LQRGainPoint* points = nullptr;
    std::size_t count = 0;
};
