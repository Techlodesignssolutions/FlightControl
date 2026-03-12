#pragma once

struct PilotInput {
    float roll_cmd = 0.0f;      // [-1, 1]
    float pitch_cmd = 0.0f;     // [-1, 1]
    float yaw_cmd = 0.0f;       // [-1, 1]
    float throttle_cmd = 0.0f;  // [0, 1]
};
