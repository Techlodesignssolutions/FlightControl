#pragma once

struct AircraftState {
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    float p_rad_s = 0.0f;
    float q_rad_s = 0.0f;
    float r_rad_s = 0.0f;

    float altitude_m = 0.0f;
    float climb_rate_mps = 0.0f;
    float airspeed_mps = 0.0f;
};
