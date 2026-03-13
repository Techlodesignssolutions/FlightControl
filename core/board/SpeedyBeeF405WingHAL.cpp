#include "SpeedyBeeF405WingHAL.h"

#include <algorithm>
#include <chrono>

namespace {
using Clock = std::chrono::steady_clock;
const Clock::time_point kStart = Clock::now();
}

SpeedyBeeF405WingHAL::SpeedyBeeF405WingHAL()
    : SpeedyBeeF405WingHAL(BoardConfig{}) {
}

SpeedyBeeF405WingHAL::SpeedyBeeF405WingHAL(const BoardConfig& config)
    : config_(config) {
    rc_channels_.fill(0.0f);
    pwm_output_us_.fill(config_.servo_neutral_us);
}

bool SpeedyBeeF405WingHAL::init() {
    initialized_ = true;
    airspeed_filter_initialized_ = false;
    climb_filter_initialized_ = false;
    return true;
}

bool SpeedyBeeF405WingHAL::readSensors(SensorData& sensor_data) {
    if (!initialized_) {
        return false;
    }

    sensor_data.roll_rad = sensor_frame_.roll_rad;
    sensor_data.pitch_rad = sensor_frame_.pitch_rad;
    sensor_data.yaw_rad = sensor_frame_.yaw_rad;

    sensor_data.p_rad_s = sensor_frame_.p_rad_s;
    sensor_data.q_rad_s = sensor_frame_.q_rad_s;
    sensor_data.r_rad_s = sensor_frame_.r_rad_s;

    sensor_data.altitude_m = sensor_frame_.altitude_m;

    if (!climb_filter_initialized_) {
        filtered_climb_rate_mps_ = sensor_frame_.climb_rate_mps;
        climb_filter_initialized_ = true;
    } else {
        const float a = clamp(config_.climb_rate_lpf_alpha, 0.0f, 1.0f);
        filtered_climb_rate_mps_ += a * (sensor_frame_.climb_rate_mps - filtered_climb_rate_mps_);
    }
    sensor_data.climb_rate_mps = filtered_climb_rate_mps_;

    if (!airspeed_filter_initialized_) {
        filtered_airspeed_mps_ = sensor_frame_.pitot_airspeed_mps;
        airspeed_filter_initialized_ = true;
    } else {
        const float a = clamp(config_.airspeed_lpf_alpha, 0.0f, 1.0f);
        filtered_airspeed_mps_ += a * (sensor_frame_.pitot_airspeed_mps - filtered_airspeed_mps_);
    }
    sensor_data.airspeed_mps = std::max(0.0f, filtered_airspeed_mps_);
    sensor_data.timestamp_us = microsNow();
    return true;
}

bool SpeedyBeeF405WingHAL::readPilotInput(PilotInput& pilot_input) {
    if (!initialized_) {
        return false;
    }

    pilot_input.roll_cmd = clamp(rc_channels_[config_.rc_roll_channel], -1.0f, 1.0f);
    pilot_input.pitch_cmd = clamp(rc_channels_[config_.rc_pitch_channel], -1.0f, 1.0f);
    pilot_input.yaw_cmd = clamp(rc_channels_[config_.rc_yaw_channel], -1.0f, 1.0f);
    pilot_input.throttle_cmd = clamp(rc_channels_[config_.rc_throttle_channel], 0.0f, 1.0f);
    return true;
}

bool SpeedyBeeF405WingHAL::writeActuators(const ActuatorCommand& cmd) {
    if (!initialized_) {
        return false;
    }

    pwm_output_us_[config_.pwm_left_elevon] = toServoPulseUs(cmd.left_elevon, config_.reverse_left_elevon);
    pwm_output_us_[config_.pwm_right_elevon] = toServoPulseUs(cmd.right_elevon, config_.reverse_right_elevon);
    pwm_output_us_[config_.pwm_rudder] = toServoPulseUs(cmd.rudder, config_.reverse_rudder);
    pwm_output_us_[config_.pwm_throttle] = toThrottlePulseUs(cmd.throttle, config_.reverse_throttle);
    return true;
}

unsigned long SpeedyBeeF405WingHAL::microsNow() {
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - kStart);
    return static_cast<unsigned long>(elapsed.count());
}

void SpeedyBeeF405WingHAL::setSensorFrame(const SensorFrame& frame) {
    sensor_frame_ = frame;
}

void SpeedyBeeF405WingHAL::setRcChannels(const std::array<float, 8>& channels_norm) {
    rc_channels_ = channels_norm;
}

float SpeedyBeeF405WingHAL::clamp(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}

float SpeedyBeeF405WingHAL::toServoPulseUs(float cmd_symm, bool reverse) const {
    float v = clamp(cmd_symm, -1.0f, 1.0f);
    if (reverse) {
        v = -v;
    }

    const float span = 0.5f * (config_.servo_max_us - config_.servo_min_us);
    return config_.servo_neutral_us + span * v;
}

float SpeedyBeeF405WingHAL::toThrottlePulseUs(float cmd_norm, bool reverse) const {
    float v = clamp(cmd_norm, 0.0f, 1.0f);
    if (reverse) {
        v = 1.0f - v;
    }
    return config_.throttle_min_us + v * (config_.throttle_max_us - config_.throttle_min_us);
}
