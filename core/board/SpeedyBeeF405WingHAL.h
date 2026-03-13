#pragma once

#include "BoardHAL.h"
#include "drivers/BaroDriver.h"
#include "drivers/ImuDriver.h"
#include "drivers/PitotDriver.h"
#include "drivers/PwmDriver.h"
#include "drivers/ReceiverDriver.h"

#include <array>
#include <cstdint>

class SpeedyBeeF405WingHAL final : public BoardHAL {
public:
    enum class HalMode {
        RealHardware,
        BenchInjected
    };

    struct BoardConfig {
        std::uint32_t serial_baud = 115200;
        HalMode mode = HalMode::RealHardware;

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

        int imu_roll_axis = 0;
        int imu_pitch_axis = 1;
        int imu_yaw_axis = 2;
        bool invert_roll_axis = false;
        bool invert_pitch_axis = false;
        bool invert_yaw_axis = false;

        float pitot_zero_offset_pa = 0.0f;
        float air_density_kg_m3 = 1.225f;

        int rc_min_us = 1000;
        int rc_mid_us = 1500;
        int rc_max_us = 2000;
        float rc_deadband = 0.03f;

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

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    void setSensorFrame(const SensorFrame& frame);
    void setRcChannels(const std::array<float, 8>& channels_norm);
#endif

private:
    bool initImu();
    bool initBaro();
    bool initPitot();
    bool initReceiver();
    bool initPwm();

    bool readImuAndAttitude(SensorFrame& frame, float dt_s);
    bool readBaro(SensorFrame& frame, float dt_s);
    bool readPitot(SensorFrame& frame);
    bool readReceiverFrame();
    bool writePwmMicros(int logical_channel, float pulse_us);

    static float clamp(float x, float lo, float hi);
    float applyDeadband(float x) const;
    float normalizeSymmetricUs(int pulse_us) const;
    float normalizeThrottleUs(int pulse_us) const;
    float toServoPulseUs(float cmd_symm, bool reverse) const;
    float toThrottlePulseUs(float cmd_norm, bool reverse) const;

    BoardConfig config_{};
    bool initialized_{false};

    unsigned long last_sensor_time_us_{0};
    bool altitude_initialized_{false};
    float last_altitude_m_{0.0f};

    SensorFrame sensor_frame_{};
    std::array<float, 8> rc_channels_{};
    std::array<float, 8> pwm_output_us_{};

    bool airspeed_filter_initialized_{false};
    float filtered_airspeed_mps_{0.0f};
    bool climb_filter_initialized_{false};
    float filtered_climb_rate_mps_{0.0f};

    ImuDriver imu_driver_{};
    BaroDriver baro_driver_{};
    PitotDriver pitot_driver_{};
    ReceiverDriver receiver_driver_{};
    PwmDriver pwm_driver_{};
};
