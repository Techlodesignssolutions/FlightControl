#include "SpeedyBeeF405WingHAL.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;
const Clock::time_point kStart = Clock::now();

}  // namespace

SpeedyBeeF405WingHAL::SpeedyBeeF405WingHAL()
    : SpeedyBeeF405WingHAL(BoardConfig{}) {
}

SpeedyBeeF405WingHAL::SpeedyBeeF405WingHAL(const BoardConfig& config)
    : config_(config) {
    radio_channels_.fill(0.0f);
    servo_outputs_.fill(0.5f);
    motor_outputs_.fill(0.0f);
}

uint32_t SpeedyBeeF405WingHAL::micros() {
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - kStart);
    return static_cast<uint32_t>(elapsed.count());
}

uint32_t SpeedyBeeF405WingHAL::millis() {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - kStart);
    return static_cast<uint32_t>(elapsed.count());
}

void SpeedyBeeF405WingHAL::delay(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void SpeedyBeeF405WingHAL::serialBegin(uint32_t baud) {
    serial_baud_ = baud;
}

void SpeedyBeeF405WingHAL::serialPrint(const char* str) {
    if (str != nullptr) {
        std::fputs(str, stdout);
    }
}

void SpeedyBeeF405WingHAL::serialPrintln(const char* str) {
    if (str != nullptr) {
        std::fputs(str, stdout);
    }
    std::fputc('\n', stdout);
}

void SpeedyBeeF405WingHAL::serialPrintFloat(float value, int decimals) {
    char format[16];
    std::snprintf(format, sizeof(format), "%%.%df", std::max(0, decimals));
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), format, value);
    serialPrint(buffer);
}

bool SpeedyBeeF405WingHAL::initIMU() {
    imu_initialized_ = true;
    return true;
}

bool SpeedyBeeF405WingHAL::readIMU(float* gyro_xyz, float* accel_xyz, float* mag_xyz) {
    if (!imu_initialized_ || gyro_xyz == nullptr || accel_xyz == nullptr || mag_xyz == nullptr) {
        return false;
    }

    gyro_xyz[0] = 0.0f;
    gyro_xyz[1] = 0.0f;
    gyro_xyz[2] = 0.0f;

    accel_xyz[0] = 0.0f;
    accel_xyz[1] = 0.0f;
    accel_xyz[2] = 9.80665f;

    mag_xyz[0] = 1.0f;
    mag_xyz[1] = 0.0f;
    mag_xyz[2] = 0.0f;
    return true;
}

bool SpeedyBeeF405WingHAL::isIMUHealthy() {
    return imu_initialized_;
}

bool SpeedyBeeF405WingHAL::initRadio() {
    radio_initialized_ = true;
    return true;
}

bool SpeedyBeeF405WingHAL::readRadio(float* channels, int num_channels) {
    if (!radio_initialized_ || channels == nullptr || num_channels <= 0) {
        return false;
    }

    const int count = std::min<int>(num_channels, static_cast<int>(radio_channels_.size()));
    for (int i = 0; i < count; ++i) {
        channels[i] = radio_channels_[static_cast<std::size_t>(i)];
    }
    return true;
}

bool SpeedyBeeF405WingHAL::isRadioConnected() {
    return radio_initialized_;
}

bool SpeedyBeeF405WingHAL::initServos() {
    servos_initialized_ = true;
    return true;
}

void SpeedyBeeF405WingHAL::writeServo(int channel, float position_0_to_1) {
    if (!servos_initialized_ || channel < 0 || channel >= static_cast<int>(servo_outputs_.size())) {
        return;
    }
    servo_outputs_[static_cast<std::size_t>(channel)] = clamp01(position_0_to_1);
}

bool SpeedyBeeF405WingHAL::initMotors() {
    motors_initialized_ = true;
    return true;
}

void SpeedyBeeF405WingHAL::writeMotor(int channel, float throttle_0_to_1) {
    if (!motors_initialized_ || channel < 0 || channel >= static_cast<int>(motor_outputs_.size())) {
        return;
    }
    motor_outputs_[static_cast<std::size_t>(channel)] = clamp01(throttle_0_to_1);
}

void SpeedyBeeF405WingHAL::digitalWrite(int, bool) {
}

bool SpeedyBeeF405WingHAL::digitalRead(int) {
    return false;
}

void SpeedyBeeF405WingHAL::pinMode(int, int) {
}

void SpeedyBeeF405WingHAL::setStatusLED(bool on) {
    status_led_ = on;
}

void SpeedyBeeF405WingHAL::blinkStatusLED(int count, int on_ms, int off_ms) {
    if (count <= 0) {
        return;
    }

    for (int i = 0; i < count; ++i) {
        setStatusLED(true);
        delay(static_cast<uint32_t>(std::max(0, on_ms)));
        setStatusLED(false);
        delay(static_cast<uint32_t>(std::max(0, off_ms)));
    }
}

float SpeedyBeeF405WingHAL::clamp01(float v) {
    return std::max(0.0f, std::min(1.0f, v));
}
