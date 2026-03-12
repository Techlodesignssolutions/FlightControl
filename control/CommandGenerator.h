#pragma once

#include "../core/ControlPipelineTypes.h"

class CommandGenerator {
public:
    struct Config {
        float max_roll_cmd_rad = 0.7f;
        float max_pitch_cmd_rad = 0.5f;
        float max_yaw_rate_cmd_rad_s = 1.57f;
    };

    CommandGenerator();
    explicit CommandGenerator(const Config& config);

    InnerLoopCommands updateFromPilot(const PilotInputs& pilot, const FlightState& state) const;
    InnerLoopCommands updateFromOuterLoop(float roll_cmd_rad,
                                          float pitch_cmd_rad,
                                          float throttle_cmd_0to1,
                                          const FlightState& state) const;

private:
    static float clampf(float x, float lo, float hi);
    Config config_{};
};
