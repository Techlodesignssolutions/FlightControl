#pragma once

#include "Ahrs.h"
#include "BoardHAL.h"

#include <array>
#include <cstdint>

class TeensyHAL final : public BoardHAL {
public:
    enum class HalError {
        None,
        InitImuFailed,
        InitReceiverFailed,
        InitOutputsFailed,
        ImuReadFailed,
        ReceiverReadFailed,
        OutputWriteFailed,
        NotInitialized
    };

    struct BoardConfig {
        std::uint32_t serial_baud = 115200;

        std::array<int, 6> rc_pins{2, 3, 4, 5, 6, 7};

        int rc_roll_channel = 1;
        int rc_pitch_channel = 2;
        int rc_yaw_channel = 3;
        int rc_throttle_channel = 0;

        std::array<int, 4> pwm_out_pins{8, 9, 10, 11};
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

        int rc_min_us = 1000;
        int rc_mid_us = 1500;
        int rc_max_us = 2000;
        float rc_deadband = 0.03f;

        float attitude_complementary_alpha = 0.98f;
    };

    TeensyHAL();
    explicit TeensyHAL(const BoardConfig& config);

    bool init() override;
    bool readSensors(SensorData& sensor_data) override;
    bool readPilotInput(PilotInput& pilot_input) override;
    bool writeActuators(const ActuatorCommand& cmd) override;
    unsigned long microsNow() override;

    HalError lastError() const { return last_error_; }

private:
    bool initImu();
    bool initReceiverPwm();
    bool initOutputs();

    bool readImuAndAttitude(SensorData& sensor_data, float dt_s);
    bool readReceiverFrame();
    bool writeServoUs(int logical_channel, float pulse_us);

    static float clamp(float x, float lo, float hi);
    float applyDeadband(float x) const;
    float normalizeSymmetricUs(int pulse_us) const;
    float normalizeThrottleUs(int pulse_us) const;
    float toServoPulseUs(float cmd_symm, bool reverse) const;
    float toThrottlePulseUs(float cmd_norm, bool reverse) const;

    static void ISR_Ch1();
    static void ISR_Ch2();
    static void ISR_Ch3();
    static void ISR_Ch4();
    static void ISR_Ch5();
    static void ISR_Ch6();

    void handleRcEdge(std::size_t idx);

    BoardConfig config_{};
    bool initialized_{false};
    HalError last_error_{HalError::None};

    bool imu_initialized_{false};
    bool receiver_initialized_{false};
    bool outputs_initialized_{false};

    float gyro_bias_[3]{0.0f, 0.0f, 0.0f};
    float accel_bias_[3]{0.0f, 0.0f, 0.0f};

    unsigned long last_sensor_time_us_{0};
    bool attitude_initialized_{false};
    Ahrs ahrs_{};

    std::array<int, 8> rc_pulse_us_{};
    std::array<float, 8> debug_last_output_us_{};

    static TeensyHAL* instance_;
    static volatile std::uint32_t ch_start_[6];
    static volatile std::uint32_t ch_width_us_[6];
};
