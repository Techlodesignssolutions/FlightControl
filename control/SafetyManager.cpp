#include "SafetyManager.h"

#include <algorithm>

SafetyManager::SafetyManager()
    : SafetyManager(Config{}) {
}

SafetyManager::SafetyManager(const Config& config)
    : config_(config) {
}

bool SafetyManager::canRunClosedLoop(const FlightState& state) const {
    return state.imu_valid && state.attitude_valid && state.radio_valid && state.actuator_valid && state.dt_s > 1e-4f && state.dt_s < 0.1f;
}

void SafetyManager::applyOutputLimits(InnerLoopOutputs& u, float dt_s) {
    const float limited_dt = std::max(dt_s, 1e-4f);
    const float max_step = config_.max_surface_rate_per_s * limited_dt;

    const float roll_limited = clampf(u.roll_surface_cmd, prev_roll_ - max_step, prev_roll_ + max_step);
    const float pitch_limited = clampf(u.pitch_surface_cmd, prev_pitch_ - max_step, prev_pitch_ + max_step);
    const float yaw_limited = clampf(u.yaw_surface_cmd, prev_yaw_ - max_step, prev_yaw_ + max_step);

    u.roll_surface_cmd = clampf(roll_limited, -config_.surface_limit, config_.surface_limit);
    u.pitch_surface_cmd = clampf(pitch_limited, -config_.surface_limit, config_.surface_limit);
    u.yaw_surface_cmd = clampf(yaw_limited, -config_.surface_limit, config_.surface_limit);
    u.throttle_cmd_0to1 = clampf(u.throttle_cmd_0to1, 0.0f, 1.0f);

    prev_roll_ = u.roll_surface_cmd;
    prev_pitch_ = u.pitch_surface_cmd;
    prev_yaw_ = u.yaw_surface_cmd;
}

float SafetyManager::safeScheduledAirspeed(const FlightState& state) {
    if (state.airspeed_valid && state.airspeed_mps > 1.0f) {
        has_last_airspeed_ = true;
        last_valid_airspeed_ = state.airspeed_mps;
        invalid_airspeed_time_s_ = 0.0f;
        return state.airspeed_mps;
    }

    invalid_airspeed_time_s_ += std::max(0.0f, state.dt_s);
    if (has_last_airspeed_ && invalid_airspeed_time_s_ <= config_.hold_last_airspeed_s) {
        return last_valid_airspeed_;
    }

    return config_.nominal_schedule_airspeed_mps;
}

float SafetyManager::clampf(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}
