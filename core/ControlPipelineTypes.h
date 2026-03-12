#pragma once

struct FlightState {
    float t_s = 0.0f;
    float dt_s = 0.0f;

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    float p_rad_s = 0.0f;
    float q_rad_s = 0.0f;
    float r_rad_s = 0.0f;

    float airspeed_mps = 0.0f;
    float altitude_m = 0.0f;
    float climb_rate_mps = 0.0f;

    bool imu_valid = false;
    bool attitude_valid = false;
    bool airspeed_valid = false;
    bool radio_valid = false;
    bool actuator_valid = false;
};

struct PilotInputs {
    float throttle_cmd_0to1 = 0.0f;
    float roll_cmd_norm = 0.0f;
    float pitch_cmd_norm = 0.0f;
    float yaw_cmd_norm = 0.0f;

    bool stabilize_mode = true;
    bool auto_throttle_mode = false;
    bool tecs_mode = false;
};

struct InnerLoopCommands {
    float roll_cmd_rad = 0.0f;
    float pitch_cmd_rad = 0.0f;
    float yaw_rate_cmd_rad_s = 0.0f;

    float throttle_cmd_0to1 = 0.0f;
};

struct InnerLoopOutputs {
    float roll_surface_cmd = 0.0f;
    float pitch_surface_cmd = 0.0f;
    float yaw_surface_cmd = 0.0f;
    float throttle_cmd_0to1 = 0.0f;
};

struct ActuatorCommands {
    float left_surface_0to1 = 0.5f;
    float right_surface_0to1 = 0.5f;
    float rudder_0to1 = 0.5f;
    float throttle_0to1 = 0.0f;
};

struct OuterLoopCommands {
    float roll_cmd_rad = 0.0f;
    float target_altitude_m = 0.0f;
    float target_airspeed_mps = 0.0f;
};

struct TECSOutputs {
    float pitch_cmd_rad = 0.0f;
    float throttle_cmd_0to1 = 0.0f;
    bool valid = false;
};
