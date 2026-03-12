#include "InnerLoopLQR.h"

#include <algorithm>

InnerLoopLQR::InnerLoopLQR(const GainProvider* gain_provider)
    : InnerLoopLQR(gain_provider, Config{}) {
}

InnerLoopLQR::InnerLoopLQR(const GainProvider* gain_provider, const Config& config)
    : gain_provider_(gain_provider)
    , config_(config) {
}

bool InnerLoopLQR::init() {
    initialized_ = gain_provider_ != nullptr;
    return initialized_;
}

void InnerLoopLQR::reset() {
}

InnerLoopOutputs InnerLoopLQR::update(const FlightState& state, const InnerLoopCommands& cmd) const {
    InnerLoopOutputs out;
    out.throttle_cmd_0to1 = clampf(cmd.throttle_cmd_0to1, 0.0f, 1.0f);

    if (!initialized_ || gain_provider_ == nullptr) {
        return out;
    }

    const float schedule_airspeed = state.airspeed_valid ? state.airspeed_mps : config_.default_schedule_airspeed_mps;
    out.roll_surface_cmd = computeRollControl(state, cmd.roll_cmd_rad, schedule_airspeed);
    out.pitch_surface_cmd = computePitchControl(state, cmd.pitch_cmd_rad, schedule_airspeed);
    out.yaw_surface_cmd = computeYawControl(state, cmd.yaw_rate_cmd_rad_s, schedule_airspeed);
    return out;
}

float InnerLoopLQR::computeRollControl(const FlightState& state, float roll_cmd_rad, float schedule_airspeed_mps) const {
    const AxisGains2State g = gain_provider_->getRollGains(schedule_airspeed_mps);
    const float error = state.roll_rad - roll_cmd_rad;
    const float u = g.trim - g.k1 * error - g.k2 * state.p_rad_s;
    return clampf(u, -config_.roll_output_limit, config_.roll_output_limit);
}

float InnerLoopLQR::computePitchControl(const FlightState& state, float pitch_cmd_rad, float schedule_airspeed_mps) const {
    const AxisGains2State g = gain_provider_->getPitchGains(schedule_airspeed_mps);
    const float error = state.pitch_rad - pitch_cmd_rad;
    const float u = g.trim - g.k1 * error - g.k2 * state.q_rad_s;
    return clampf(u, -config_.pitch_output_limit, config_.pitch_output_limit);
}

float InnerLoopLQR::computeYawControl(const FlightState& state, float yaw_rate_cmd_rad_s, float schedule_airspeed_mps) const {
    const AxisGains2State g = gain_provider_->getYawGains(schedule_airspeed_mps);
    const float error = state.r_rad_s - yaw_rate_cmd_rad_s;
    const float u = g.trim - g.k1 * error - g.k2 * state.r_rad_s;
    return clampf(u, -config_.yaw_output_limit, config_.yaw_output_limit);
}

float InnerLoopLQR::clampf(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}
