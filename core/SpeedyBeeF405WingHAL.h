#pragma once

#include "HAL.h"

#include <array>
#include <cstdint>

/**
 * HAL implementation scaffold for the SpeedyBee F405 Wing flight controller.
 *
 * Notes:
 * - This is a board-targeted integration layer intended to be connected to
 *   your firmware SDK / drivers (e.g. STM32 HAL, INAV/ArduPilot board support).
 * - Sensor and RC read methods are implemented as safe stubs so the control
 *   stack can be built and unit-tested before full hardware bring-up.
 */
class SpeedyBeeF405WingHAL final : public HAL {
public:
    struct BoardConfig {
        std::uint32_t serial_baud = 115200;
        int status_led_pin = 0;

        // Default fixed-wing control outputs for elevon + rudder + throttle.
        // These indexes are logical channels consumed by the mixer.
        int servo_channel_left_elevon = 0;
        int servo_channel_right_elevon = 1;
        int servo_channel_rudder = 2;
        int motor_channel_throttle = 0;

        // RC expectations (SBUS/CRSF mapped to normalized channels in readRadio).
        int radio_channel_throttle = 0;
        int radio_channel_roll = 1;
        int radio_channel_pitch = 2;
        int radio_channel_yaw = 3;
        int radio_channel_aux1 = 4;
        int radio_channel_aux2 = 5;
    };

    SpeedyBeeF405WingHAL();
    explicit SpeedyBeeF405WingHAL(const BoardConfig& config);

    uint32_t micros() override;
    uint32_t millis() override;
    void delay(uint32_t ms) override;

    void serialBegin(uint32_t baud) override;
    void serialPrint(const char* str) override;
    void serialPrintln(const char* str) override;
    void serialPrintFloat(float value, int decimals = 2) override;

    bool initIMU() override;
    bool readIMU(float* gyro_xyz, float* accel_xyz, float* mag_xyz) override;
    bool isIMUHealthy() override;

    bool initRadio() override;
    bool readRadio(float* channels, int num_channels) override;
    bool isRadioConnected() override;

    bool initAirspeed() override;
    bool readAirspeed(float* airspeed_mps) override;
    bool isAirspeedHealthy() override;

    bool initServos() override;
    void writeServo(int channel, float position_0_to_1) override;

    bool initMotors() override;
    void writeMotor(int channel, float throttle_0_to_1) override;

    void digitalWrite(int pin, bool high) override;
    bool digitalRead(int pin) override;
    void pinMode(int pin, int mode) override;

    void setStatusLED(bool on) override;
    void blinkStatusLED(int count, int on_ms, int off_ms) override;

private:
    static float clamp01(float v);

    BoardConfig config_{};

    bool imu_initialized_{false};
    bool radio_initialized_{false};
    bool airspeed_initialized_{false};
    bool servos_initialized_{false};
    bool motors_initialized_{false};

    std::array<float, 8> radio_channels_{};
    std::array<float, 8> servo_outputs_{};
    float airspeed_mps_{0.0f};
    std::array<float, 4> motor_outputs_{};

    bool status_led_{false};
    std::uint32_t serial_baud_{0};
};
