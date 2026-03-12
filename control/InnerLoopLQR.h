#pragma once

#include "GainProvider.h"
#include "../core/ControlPipelineTypes.h"

class InnerLoopLQR {
public:
    struct Config {
        float roll_output_limit = 1.0f;
        float pitch_output_limit = 1.0f;
        float yaw_output_limit = 1.0f;
        float default_schedule_airspeed_mps = 15.0f;
    };

    explicit InnerLoopLQR(const GainProvider* gain_provider);
    InnerLoopLQR(const GainProvider* gain_provider, const Config& config);

    bool init();
    void reset();

    InnerLoopOutputs update(const FlightState& state, const InnerLoopCommands& cmd) const;

private:
    float computeRollControl(const FlightState& state, float roll_cmd_rad, float schedule_airspeed_mps) const;
    float computePitchControl(const FlightState& state, float pitch_cmd_rad, float schedule_airspeed_mps) const;
    float computeYawControl(const FlightState& state, float yaw_rate_cmd_rad_s, float schedule_airspeed_mps) const;
    static float clampf(float x, float lo, float hi);

    const GainProvider* gain_provider_{nullptr};
    Config config_{};
    bool initialized_{false};
};
