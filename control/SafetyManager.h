#pragma once

#include "../core/ControlPipelineTypes.h"

class SafetyManager {
public:
    struct Config {
        float surface_limit = 1.0f;
        float max_surface_rate_per_s = 3.0f;
        float nominal_schedule_airspeed_mps = 15.0f;
        float hold_last_airspeed_s = 0.75f;
    };

    SafetyManager();
    explicit SafetyManager(const Config& config);

    bool canRunClosedLoop(const FlightState& state) const;
    void applyOutputLimits(InnerLoopOutputs& u, float dt_s);
    float safeScheduledAirspeed(const FlightState& state);

private:
    static float clampf(float x, float lo, float hi);

    Config config_{};
    bool has_last_airspeed_{false};
    float last_valid_airspeed_{0.0f};
    float invalid_airspeed_time_s_{0.0f};

    float prev_roll_{0.0f};
    float prev_pitch_{0.0f};
    float prev_yaw_{0.0f};
};
