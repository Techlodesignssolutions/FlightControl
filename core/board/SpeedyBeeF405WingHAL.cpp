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
    pilot_channels_.fill(0.0f);
}

bool SpeedyBeeF405WingHAL::init() {
    initialized_ = true;
    return true;
}

bool SpeedyBeeF405WingHAL::readSensors(SensorData& sensor_data) {
    if (!initialized_) {
        return false;
    }

    sensor_data.roll_rad = 0.0f;
    sensor_data.pitch_rad = 0.0f;
    sensor_data.yaw_rad = 0.0f;
    sensor_data.p_rad_s = 0.0f;
    sensor_data.q_rad_s = 0.0f;
    sensor_data.r_rad_s = 0.0f;
    sensor_data.altitude_m = 0.0f;
    sensor_data.climb_rate_mps = 0.0f;
    sensor_data.airspeed_mps = 15.0f;
    sensor_data.timestamp_us = microsNow();
    return true;
}

bool SpeedyBeeF405WingHAL::readPilotInput(PilotInput& pilot_input) {
    if (!initialized_) {
        return false;
    }

    pilot_input.roll_cmd = clamp(pilot_channels_[0], -1.0f, 1.0f);
    pilot_input.pitch_cmd = clamp(pilot_channels_[1], -1.0f, 1.0f);
    pilot_input.yaw_cmd = clamp(pilot_channels_[2], -1.0f, 1.0f);
    pilot_input.throttle_cmd = clamp(pilot_channels_[3], 0.0f, 1.0f);
    return true;
}

bool SpeedyBeeF405WingHAL::writeActuators(const ActuatorCommand& cmd) {
    if (!initialized_) {
        return false;
    }

    last_actuator_cmd_.left_elevon = clamp(cmd.left_elevon, -1.0f, 1.0f);
    last_actuator_cmd_.right_elevon = clamp(cmd.right_elevon, -1.0f, 1.0f);
    last_actuator_cmd_.rudder = clamp(cmd.rudder, -1.0f, 1.0f);
    last_actuator_cmd_.throttle = clamp(cmd.throttle, 0.0f, 1.0f);
    return true;
}

unsigned long SpeedyBeeF405WingHAL::microsNow() {
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - kStart);
    return static_cast<unsigned long>(elapsed.count());
}

float SpeedyBeeF405WingHAL::clamp(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}
