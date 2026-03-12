#pragma once

struct AirspeedEstimate {
    float dynamic_pressure_pa = 0.0f;
    float airspeed_mps = 0.0f;
    bool valid = false;
};

class AirspeedSensor {
public:
    bool init();
    AirspeedEstimate update(float dt_s, float differential_pressure_pa);

private:
    float filtered_pressure_pa_ = 0.0f;
    bool initialized_ = false;
};
