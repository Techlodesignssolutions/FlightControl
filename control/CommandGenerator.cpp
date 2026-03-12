#include "CommandGenerator.h"

#include <algorithm>

CommandGenerator::CommandGenerator()
    : CommandGenerator(Config{}) {
}

CommandGenerator::CommandGenerator(const Config& config)
    : config_(config) {
}

InnerLoopCommands CommandGenerator::updateFromPilot(const PilotInputs& pilot, const FlightState&) const {
    InnerLoopCommands cmd;
    cmd.roll_cmd_rad = clampf(pilot.roll_cmd_norm, -1.0f, 1.0f) * config_.max_roll_cmd_rad;
    cmd.pitch_cmd_rad = clampf(pilot.pitch_cmd_norm, -1.0f, 1.0f) * config_.max_pitch_cmd_rad;
    cmd.yaw_rate_cmd_rad_s = clampf(pilot.yaw_cmd_norm, -1.0f, 1.0f) * config_.max_yaw_rate_cmd_rad_s;
    cmd.throttle_cmd_0to1 = clampf(pilot.throttle_cmd_0to1, 0.0f, 1.0f);
    return cmd;
}

InnerLoopCommands CommandGenerator::updateFromOuterLoop(float roll_cmd_rad,
                                                        float pitch_cmd_rad,
                                                        float throttle_cmd_0to1,
                                                        const FlightState&) const {
    InnerLoopCommands cmd;
    cmd.roll_cmd_rad = clampf(roll_cmd_rad, -config_.max_roll_cmd_rad, config_.max_roll_cmd_rad);
    cmd.pitch_cmd_rad = clampf(pitch_cmd_rad, -config_.max_pitch_cmd_rad, config_.max_pitch_cmd_rad);
    cmd.yaw_rate_cmd_rad_s = 0.0f;
    cmd.throttle_cmd_0to1 = clampf(throttle_cmd_0to1, 0.0f, 1.0f);
    return cmd;
}

float CommandGenerator::clampf(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}
