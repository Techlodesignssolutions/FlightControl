#pragma once

struct AxisGains2State {
    float k1 = 0.0f;
    float k2 = 0.0f;
    float trim = 0.0f;
};

class GainProvider {
public:
    virtual ~GainProvider() = default;

    virtual AxisGains2State getRollGains(float airspeed_mps) const = 0;
    virtual AxisGains2State getPitchGains(float airspeed_mps) const = 0;
    virtual AxisGains2State getYawGains(float airspeed_mps) const = 0;
};
