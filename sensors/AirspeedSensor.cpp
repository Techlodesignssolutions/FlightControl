#include "AirspeedSensor.h"

#include <algorithm>
#include <cmath>

bool AirspeedSensor::init() {
    initialized_ = true;
    filtered_pressure_pa_ = 0.0f;
    return true;
}

AirspeedEstimate AirspeedSensor::update(float dt_s, float differential_pressure_pa) {
    AirspeedEstimate out;
    if (!initialized_ || dt_s <= 0.0f) {
        return out;
    }

    const float alpha = std::max(0.0f, std::min(1.0f, dt_s * 8.0f));
    filtered_pressure_pa_ += alpha * (differential_pressure_pa - filtered_pressure_pa_);

    constexpr float kRho = 1.225f;
    out.dynamic_pressure_pa = std::max(0.0f, filtered_pressure_pa_);
    out.airspeed_mps = std::sqrt((2.0f * out.dynamic_pressure_pa) / kRho);
    out.valid = std::isfinite(out.airspeed_mps) != 0 && out.airspeed_mps >= 0.0f && out.airspeed_mps < 120.0f;
    return out;
}
