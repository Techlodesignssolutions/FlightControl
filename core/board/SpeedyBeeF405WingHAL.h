#pragma once

#include "BoardHAL.h"

#include <array>
#include <cstdint>

class SpeedyBeeF405WingHAL final : public BoardHAL {
public:
    struct BoardConfig {
        std::uint32_t serial_baud = 115200;

        int rc_roll_channel = 0;
        int rc_pitch_channel = 1;
        int rc_yaw_channel = 2;
        int rc_throttle_channel = 3;

        int pwm_left_elevon = 0;
        int pwm_right_elevon = 1;
        int pwm_rudder = 2;
        int pwm_throttle = 3;

        float servo_neutral_us = 1500.0f;
        float servo_min_us = 1000.0f;
        float servo_max_us = 2000.0f;
        float throttle_min_us = 1000.0f;
        float throttle_max_us = 2000.0f;

        bool reverse_left_elevon = false;
        bool reverse_right_elevon = false;
        bool reverse_rudder = false;
        bool reverse_throttle = false;

        float airspeed_lpf_alpha = 0.15f;
        float climb_rate_lpf_alpha = 0.2f;
    };

    struct SensorFrame {
        float roll_rad = 0.0f;
        float pitch_rad = 0.0f;
        float yaw_rad = 0.0f;
        float p_rad_s = 0.0f;
        float q_rad_s = 0.0f;
        float r_rad_s = 0.0f;
        float altitude_m = 0.0f;
        float climb_rate_mps = 0.0f;
        float pitot_airspeed_mps = 0.0f;
    };

    SpeedyBeeF405WingHAL();
    explicit SpeedyBeeF405WingHAL(const BoardConfig& config);

    bool init() override;
    bool readSensors(SensorData& sensor_data) override;
    bool readPilotInput(PilotInput& pilot_input) override;
    bool writeActuators(const ActuatorCommand& cmd) override;
    unsigned long microsNow() override;

    // Integration hooks for board drivers / bench harness.
    void setSensorFrame(const SensorFrame& frame);
    void setRcChannels(const std::array<float, 8>& channels_norm);

private:
    static float clamp(float x, float lo, float hi);
    float toServoPulseUs(float cmd_symm, bool reverse) const;
    float toThrottlePulseUs(float cmd_norm, bool reverse) const;

    BoardConfig config_{};
    bool initialized_{false};

    SensorFrame sensor_frame_{};
    std::array<float, 8> rc_channels_{};
    std::array<float, 8> pwm_output_us_{};

    bool airspeed_filter_initialized_{false};
    float filtered_airspeed_mps_{0.0f};
    bool climb_filter_initialized_{false};
    float filtered_climb_rate_mps_{0.0f};
};
