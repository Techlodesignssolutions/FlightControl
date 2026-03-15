#include "TeensyHAL.h"

#include <cmath>

#if defined(ARDUINO)
#include <Arduino.h>
#include <PWMServo.h>
#include <Wire.h>

#include <MPU6050.h>
#endif

TeensyHAL* TeensyHAL::instance_ = nullptr;
volatile std::uint32_t TeensyHAL::ch_start_[6] = {0, 0, 0, 0, 0, 0};
volatile std::uint32_t TeensyHAL::ch_width_us_[6] = {1500, 1500, 1500, 1500, 1500, 1500};

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kG = 9.80665f;
#if defined(ARDUINO)
MPU6050 g_mpu6050;
PWMServo g_outputs[4];
#endif
}

TeensyHAL::TeensyHAL() : TeensyHAL(BoardConfig{}) {}

TeensyHAL::TeensyHAL(const BoardConfig& config) : config_(config) {
    rc_pulse_us_.fill(config_.rc_mid_us);
    rc_pulse_us_[config_.rc_throttle_channel] = config_.rc_min_us;
    debug_last_output_us_.fill(config_.servo_neutral_us);
}

bool TeensyHAL::init() {
#if !defined(ARDUINO)
    last_error_ = HalError::NotInitialized;
    return false;
#else
    Serial.begin(config_.serial_baud);
    delay(50);

    if (!initImu()) {
        last_error_ = HalError::InitImuFailed;
        return false;
    }

    if (!initReceiverPwm()) {
        last_error_ = HalError::InitReceiverFailed;
        return false;
    }

    if (!initOutputs()) {
        last_error_ = HalError::InitOutputsFailed;
        return false;
    }

    last_sensor_time_us_ = microsNow();
    attitude_initialized_ = false;
    ahrs_.reset();
    initialized_ = true;
    last_error_ = HalError::None;
    return true;
#endif
}

bool TeensyHAL::initImu() {
#if !defined(ARDUINO)
    return false;
#else
    Wire.begin();
    Wire.setClock(1000000);

    g_mpu6050.initialize();
    if (!g_mpu6050.testConnection()) {
        return false;
    }

    g_mpu6050.setFullScaleGyroRange(0);   // 250 dps
    g_mpu6050.setFullScaleAccelRange(0);  // 2g

    constexpr int kSamples = 1200;
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < kSamples; ++i) {
        int16_t ax, ay, az, gx, gy, gz;
        g_mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        gyro_sum[0] += static_cast<float>(gx) / 131.0f;
        gyro_sum[1] += static_cast<float>(gy) / 131.0f;
        gyro_sum[2] += static_cast<float>(gz) / 131.0f;

        accel_sum[0] += static_cast<float>(ax) / 16384.0f;
        accel_sum[1] += static_cast<float>(ay) / 16384.0f;
        accel_sum[2] += static_cast<float>(az) / 16384.0f;
        delay(1);
    }

    gyro_bias_[0] = gyro_sum[0] / static_cast<float>(kSamples);
    gyro_bias_[1] = gyro_sum[1] / static_cast<float>(kSamples);
    gyro_bias_[2] = gyro_sum[2] / static_cast<float>(kSamples);

    accel_bias_[0] = accel_sum[0] / static_cast<float>(kSamples);
    accel_bias_[1] = accel_sum[1] / static_cast<float>(kSamples);
    accel_bias_[2] = accel_sum[2] / static_cast<float>(kSamples) - 1.0f;

    imu_initialized_ = true;
    return true;
#endif
}

bool TeensyHAL::readImuAndAttitude(SensorData& sensor_data, float dt_s) {
#if !defined(ARDUINO)
    (void)sensor_data;
    (void)dt_s;
    return false;
#else
    if (!imu_initialized_) {
        return false;
    }

    int16_t ax, ay, az, gx, gy, gz;
    g_mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    const float accel_x_ms2 = ((static_cast<float>(ax) / 16384.0f) - accel_bias_[0]) * kG;
    const float accel_y_ms2 = ((static_cast<float>(ay) / 16384.0f) - accel_bias_[1]) * kG;
    const float accel_z_ms2 = ((static_cast<float>(az) / 16384.0f) - accel_bias_[2]) * kG;

    const float p = ((static_cast<float>(gx) / 131.0f) - gyro_bias_[0]) * kDegToRad;
    const float q = ((static_cast<float>(gy) / 131.0f) - gyro_bias_[1]) * kDegToRad;
    const float r = ((static_cast<float>(gz) / 131.0f) - gyro_bias_[2]) * kDegToRad;

    ahrs_.update(p, q, r, accel_x_ms2, accel_y_ms2, accel_z_ms2, dt_s,
                 config_.attitude_complementary_alpha);

    sensor_data.roll = ahrs_.roll();
    sensor_data.pitch = ahrs_.pitch();
    sensor_data.yaw = ahrs_.yaw();
    sensor_data.p = p;
    sensor_data.q = q;
    sensor_data.r = r;

    sensor_data.altitude = 0.0f;
    sensor_data.climb_rate = 0.0f;
    sensor_data.pitot_airspeed = 0.0f;

    attitude_initialized_ = true;
    return true;
#endif
}

bool TeensyHAL::readSensors(SensorData& sensor_data) {
    if (!initialized_) {
        last_error_ = HalError::NotInitialized;
        return false;
    }

    const unsigned long now_us = microsNow();
    float dt_s = static_cast<float>(now_us - last_sensor_time_us_) * 1.0e-6f;
    if (last_sensor_time_us_ == 0 || dt_s <= 0.0f || dt_s > 0.1f) {
        dt_s = 0.002f;
    }
    last_sensor_time_us_ = now_us;

    if (!readImuAndAttitude(sensor_data, dt_s)) {
        last_error_ = HalError::ImuReadFailed;
        return false;
    }

    last_error_ = HalError::None;
    return true;
}

bool TeensyHAL::initReceiverPwm() {
#if !defined(ARDUINO)
    return false;
#else
    instance_ = this;
    for (std::size_t i = 0; i < 6; ++i) {
        pinMode(config_.rc_pins[i], INPUT_PULLUP);
        ch_width_us_[i] = static_cast<std::uint32_t>(config_.rc_mid_us);
        ch_start_[i] = 0;
    }
    ch_width_us_[config_.rc_throttle_channel] = static_cast<std::uint32_t>(config_.rc_min_us);

    attachInterrupt(digitalPinToInterrupt(config_.rc_pins[0]), ISR_Ch1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.rc_pins[1]), ISR_Ch2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.rc_pins[2]), ISR_Ch3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.rc_pins[3]), ISR_Ch4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.rc_pins[4]), ISR_Ch5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.rc_pins[5]), ISR_Ch6, CHANGE);

    receiver_initialized_ = true;
    return true;
#endif
}

bool TeensyHAL::readReceiverFrame() {
    if (!receiver_initialized_) {
        return false;
    }

#if defined(ARDUINO)
    noInterrupts();
    for (std::size_t i = 0; i < 6; ++i) {
        rc_pulse_us_[i] = static_cast<int>(ch_width_us_[i]);
    }
    interrupts();
#endif

    for (std::size_t i = 0; i < 6; ++i) {
        rc_pulse_us_[i] = static_cast<int>(clamp(static_cast<float>(rc_pulse_us_[i]),
                                                 static_cast<float>(config_.rc_min_us),
                                                 static_cast<float>(config_.rc_max_us)));
    }
    return true;
}

bool TeensyHAL::readPilotInput(PilotInput& pilot_input) {
    if (!initialized_) {
        last_error_ = HalError::NotInitialized;
        return false;
    }

    if (!readReceiverFrame()) {
        last_error_ = HalError::ReceiverReadFailed;
        return false;
    }

    pilot_input.roll = applyDeadband(normalizeSymmetricUs(rc_pulse_us_[config_.rc_roll_channel]));
    pilot_input.pitch = applyDeadband(normalizeSymmetricUs(rc_pulse_us_[config_.rc_pitch_channel]));
    pilot_input.yaw = applyDeadband(normalizeSymmetricUs(rc_pulse_us_[config_.rc_yaw_channel]));
    pilot_input.throttle = normalizeThrottleUs(rc_pulse_us_[config_.rc_throttle_channel]);
    pilot_input.armed = (pilot_input.throttle < 0.05f) && (rc_pulse_us_[4] > config_.rc_mid_us);

    last_error_ = HalError::None;
    return true;
}

bool TeensyHAL::initOutputs() {
#if !defined(ARDUINO)
    return false;
#else
    for (std::size_t i = 0; i < 4; ++i) {
        g_outputs[i].attach(config_.pwm_out_pins[i], config_.servo_min_us, config_.servo_max_us);
    }

    outputs_initialized_ = true;

    const bool ok =
        writeServoUs(config_.pwm_left_elevon, config_.servo_neutral_us) &&
        writeServoUs(config_.pwm_right_elevon, config_.servo_neutral_us) &&
        writeServoUs(config_.pwm_rudder, config_.servo_neutral_us) &&
        writeServoUs(config_.pwm_throttle, config_.throttle_min_us);

    if (!ok) {
        outputs_initialized_ = false;
        return false;
    }

    return true;
#endif
}

bool TeensyHAL::writeServoUs(int logical_channel, float pulse_us) {
    if (!outputs_initialized_) {
        return false;
    }

#if defined(ARDUINO)
    if (logical_channel < 0 || logical_channel >= 4) {
        return false;
    }

    const float clamped = clamp(pulse_us, config_.servo_min_us, config_.servo_max_us);
    g_outputs[logical_channel].writeMicroseconds(static_cast<int>(clamped));
    debug_last_output_us_[logical_channel] = clamped;
    return true;
#else
    (void)logical_channel;
    (void)pulse_us;
    return false;
#endif
}

bool TeensyHAL::writeActuators(const ActuatorCommand& cmd) {
    if (!initialized_ || !outputs_initialized_) {
        last_error_ = HalError::OutputWriteFailed;
        return false;
    }

    const float left_us = toServoPulseUs(cmd.left_elevon, config_.reverse_left_elevon);
    const float right_us = toServoPulseUs(cmd.right_elevon, config_.reverse_right_elevon);
    const float rudder_us = toServoPulseUs(cmd.rudder, config_.reverse_rudder);
    const float throttle_us = toThrottlePulseUs(cmd.throttle, config_.reverse_throttle);

    const bool ok = writeServoUs(config_.pwm_left_elevon, left_us) &&
                    writeServoUs(config_.pwm_right_elevon, right_us) &&
                    writeServoUs(config_.pwm_rudder, rudder_us) &&
                    writeServoUs(config_.pwm_throttle, throttle_us);

    if (!ok) {
        last_error_ = HalError::OutputWriteFailed;
        return false;
    }

    last_error_ = HalError::None;
    return true;
}

unsigned long TeensyHAL::microsNow() {
#if defined(ARDUINO)
    return micros();
#else
    return 0;
#endif
}

float TeensyHAL::clamp(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float TeensyHAL::applyDeadband(float x) const {
    const float db = config_.rc_deadband;
    if (std::fabs(x) <= db) {
        return 0.0f;
    }

    if (x > 0.0f) {
        return (x - db) / (1.0f - db);
    }

    return (x + db) / (1.0f - db);
}

float TeensyHAL::normalizeSymmetricUs(int pulse_us) const {
    const float centered = static_cast<float>(pulse_us - config_.rc_mid_us);
    const float half_span = static_cast<float>(config_.rc_max_us - config_.rc_mid_us);
    return clamp(centered / half_span, -1.0f, 1.0f);
}

float TeensyHAL::normalizeThrottleUs(int pulse_us) const {
    const float span = static_cast<float>(config_.rc_max_us - config_.rc_min_us);
    return clamp((static_cast<float>(pulse_us - config_.rc_min_us)) / span, 0.0f, 1.0f);
}

float TeensyHAL::toServoPulseUs(float cmd_symm, bool reverse) const {
    const float c = reverse ? -cmd_symm : cmd_symm;
    const float span = (config_.servo_max_us - config_.servo_min_us) * 0.5f;
    return clamp(config_.servo_neutral_us + c * span, config_.servo_min_us, config_.servo_max_us);
}

float TeensyHAL::toThrottlePulseUs(float cmd_norm, bool reverse) const {
    const float c = reverse ? (1.0f - cmd_norm) : cmd_norm;
    return clamp(config_.throttle_min_us + c * (config_.throttle_max_us - config_.throttle_min_us),
                 config_.throttle_min_us,
                 config_.throttle_max_us);
}

void TeensyHAL::handleRcEdge(std::size_t idx) {
#if defined(ARDUINO)
    if (!instance_) {
        return;
    }

    const bool level = digitalRead(instance_->config_.rc_pins[idx]) == HIGH;
    const std::uint32_t now = micros();
    if (level) {
        ch_start_[idx] = now;
    } else {
        ch_width_us_[idx] = now - ch_start_[idx];
    }
#else
    (void)idx;
#endif
}

void TeensyHAL::ISR_Ch1() { if (instance_) instance_->handleRcEdge(0); }
void TeensyHAL::ISR_Ch2() { if (instance_) instance_->handleRcEdge(1); }
void TeensyHAL::ISR_Ch3() { if (instance_) instance_->handleRcEdge(2); }
void TeensyHAL::ISR_Ch4() { if (instance_) instance_->handleRcEdge(3); }
void TeensyHAL::ISR_Ch5() { if (instance_) instance_->handleRcEdge(4); }
void TeensyHAL::ISR_Ch6() { if (instance_) instance_->handleRcEdge(5); }
