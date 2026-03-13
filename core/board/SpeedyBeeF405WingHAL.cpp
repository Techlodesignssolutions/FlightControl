#include "SpeedyBeeF405WingHAL.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {
using Clock = std::chrono::steady_clock;
const Clock::time_point kStart = Clock::now();
}

SpeedyBeeF405WingHAL::SpeedyBeeF405WingHAL()
    : SpeedyBeeF405WingHAL(BoardConfig{}) {
}

SpeedyBeeF405WingHAL::SpeedyBeeF405WingHAL(const BoardConfig& config)
    : config_(config) {
    rc_channels_.fill(0.0f);
    pwm_output_us_.fill(config_.servo_neutral_us);
}

bool SpeedyBeeF405WingHAL::init() {
    bool ok = true;
    ok = ok && initImu();
    ok = ok && initBaro();
    ok = ok && initPitot();
    ok = ok && initReceiver();
    ok = ok && initPwm();

    initialized_ = ok;
    airspeed_filter_initialized_ = false;
    climb_filter_initialized_ = false;
    altitude_initialized_ = false;
    last_sensor_time_us_ = microsNow();
    return ok;
}

bool SpeedyBeeF405WingHAL::readSensors(SensorData& sensor_data) {
    if (!initialized_) {
        return false;
    }

    const unsigned long now_us = microsNow();
    const float dt_s = std::max(0.0f, (now_us - last_sensor_time_us_) * 1e-6f);
    last_sensor_time_us_ = now_us;

    SensorFrame frame{};
    if (!readImuAndAttitude(frame, dt_s)) {
        return false;
    }
    if (!readBaro(frame, dt_s)) {
        return false;
    }
    if (!readPitot(frame)) {
        return false;
    }

    sensor_data.roll_rad = frame.roll_rad;
    sensor_data.pitch_rad = frame.pitch_rad;
    sensor_data.yaw_rad = frame.yaw_rad;
    sensor_data.p_rad_s = frame.p_rad_s;
    sensor_data.q_rad_s = frame.q_rad_s;
    sensor_data.r_rad_s = frame.r_rad_s;
    sensor_data.altitude_m = frame.altitude_m;

    if (!climb_filter_initialized_) {
        filtered_climb_rate_mps_ = frame.climb_rate_mps;
        climb_filter_initialized_ = true;
    } else {
        const float a = clamp(config_.climb_rate_lpf_alpha, 0.0f, 1.0f);
        filtered_climb_rate_mps_ += a * (frame.climb_rate_mps - filtered_climb_rate_mps_);
    }
    sensor_data.climb_rate_mps = filtered_climb_rate_mps_;

    if (!airspeed_filter_initialized_) {
        filtered_airspeed_mps_ = frame.pitot_airspeed_mps;
        airspeed_filter_initialized_ = true;
    } else {
        const float a = clamp(config_.airspeed_lpf_alpha, 0.0f, 1.0f);
        filtered_airspeed_mps_ += a * (frame.pitot_airspeed_mps - filtered_airspeed_mps_);
    }
    sensor_data.airspeed_mps = std::max(0.0f, filtered_airspeed_mps_);
    sensor_data.timestamp_us = now_us;

    sensor_frame_ = frame;
    return true;
}

bool SpeedyBeeF405WingHAL::readPilotInput(PilotInput& pilot_input) {
    if (!initialized_ || !readReceiverFrame()) {
        return false;
    }

    pilot_input.roll_cmd = clamp(rc_channels_[config_.rc_roll_channel], -1.0f, 1.0f);
    pilot_input.pitch_cmd = clamp(rc_channels_[config_.rc_pitch_channel], -1.0f, 1.0f);
    pilot_input.yaw_cmd = clamp(rc_channels_[config_.rc_yaw_channel], -1.0f, 1.0f);
    pilot_input.throttle_cmd = clamp(rc_channels_[config_.rc_throttle_channel], 0.0f, 1.0f);
    return true;
}

bool SpeedyBeeF405WingHAL::writeActuators(const ActuatorCommand& cmd) {
    if (!initialized_) {
        return false;
    }

    const float left_us = toServoPulseUs(cmd.left_elevon, config_.reverse_left_elevon);
    const float right_us = toServoPulseUs(cmd.right_elevon, config_.reverse_right_elevon);
    const float rudder_us = toServoPulseUs(cmd.rudder, config_.reverse_rudder);
    const float throttle_us = toThrottlePulseUs(cmd.throttle, config_.reverse_throttle);

    bool ok = true;
    ok = ok && writePwmMicros(config_.pwm_left_elevon, left_us);
    ok = ok && writePwmMicros(config_.pwm_right_elevon, right_us);
    ok = ok && writePwmMicros(config_.pwm_rudder, rudder_us);
    ok = ok && writePwmMicros(config_.pwm_throttle, throttle_us);

    // Keep mirror cache for bench visibility.
    pwm_output_us_[config_.pwm_left_elevon] = left_us;
    pwm_output_us_[config_.pwm_right_elevon] = right_us;
    pwm_output_us_[config_.pwm_rudder] = rudder_us;
    pwm_output_us_[config_.pwm_throttle] = throttle_us;
    return ok;
}

unsigned long SpeedyBeeF405WingHAL::microsNow() {
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - kStart);
    return static_cast<unsigned long>(elapsed.count());
}

bool SpeedyBeeF405WingHAL::initImu() {
    return imu_driver_.init();
}

bool SpeedyBeeF405WingHAL::initBaro() {
    return baro_driver_.init();
}

bool SpeedyBeeF405WingHAL::initPitot() {
    return pitot_driver_.init();
}

bool SpeedyBeeF405WingHAL::initReceiver() {
    return receiver_driver_.init();
}

bool SpeedyBeeF405WingHAL::initPwm() {
    return pwm_driver_.init();
}

bool SpeedyBeeF405WingHAL::readImuAndAttitude(SensorFrame& frame, float) {
    ImuSample raw{};
    if (!imu_driver_.read(raw)) {
        return false;
    }

    const float sign_roll = config_.invert_roll_axis ? -1.0f : 1.0f;
    const float sign_pitch = config_.invert_pitch_axis ? -1.0f : 1.0f;
    const float sign_yaw = config_.invert_yaw_axis ? -1.0f : 1.0f;

    const float g[3] = {raw.gx_rad_s, raw.gy_rad_s, raw.gz_rad_s};
    const float a[3] = {raw.roll_rad, raw.pitch_rad, raw.yaw_rad};

    const int roll_i = std::max(0, std::min(2, config_.imu_roll_axis));
    const int pitch_i = std::max(0, std::min(2, config_.imu_pitch_axis));
    const int yaw_i = std::max(0, std::min(2, config_.imu_yaw_axis));

    frame.p_rad_s = sign_roll * g[roll_i];
    frame.q_rad_s = sign_pitch * g[pitch_i];
    frame.r_rad_s = sign_yaw * g[yaw_i];

    frame.roll_rad = sign_roll * a[roll_i];
    frame.pitch_rad = sign_pitch * a[pitch_i];
    frame.yaw_rad = sign_yaw * a[yaw_i];
    return true;
}

bool SpeedyBeeF405WingHAL::readBaro(SensorFrame& frame, float dt_s) {
    float altitude_m = 0.0f;
    if (!baro_driver_.readAltitudeMeters(altitude_m)) {
        return false;
    }

    frame.altitude_m = altitude_m;
    if (!altitude_initialized_ || dt_s <= 0.0f) {
        frame.climb_rate_mps = 0.0f;
        last_altitude_m_ = altitude_m;
        altitude_initialized_ = true;
        return true;
    }

    frame.climb_rate_mps = (altitude_m - last_altitude_m_) / dt_s;
    last_altitude_m_ = altitude_m;
    return true;
}

bool SpeedyBeeF405WingHAL::readPitot(SensorFrame& frame) {
    float dp_pa = 0.0f;
    if (!pitot_driver_.readDifferentialPressurePa(dp_pa)) {
        return false;
    }

    dp_pa -= config_.pitot_zero_offset_pa;
    if (dp_pa < 0.0f) {
        dp_pa = 0.0f;
    }

    const float rho = std::max(0.5f, config_.air_density_kg_m3);
    frame.pitot_airspeed_mps = std::sqrt((2.0f * dp_pa) / rho);
    return true;
}

bool SpeedyBeeF405WingHAL::readReceiverFrame() {
    std::array<int, 8> pulses_us{};
    if (!receiver_driver_.readPulsesUs(pulses_us)) {
        return false;
    }

    rc_channels_[config_.rc_roll_channel] = normalizeSymmetricUs(pulses_us[config_.rc_roll_channel]);
    rc_channels_[config_.rc_pitch_channel] = normalizeSymmetricUs(pulses_us[config_.rc_pitch_channel]);
    rc_channels_[config_.rc_yaw_channel] = normalizeSymmetricUs(pulses_us[config_.rc_yaw_channel]);
    rc_channels_[config_.rc_throttle_channel] = normalizeThrottleUs(pulses_us[config_.rc_throttle_channel]);
    return true;
}

bool SpeedyBeeF405WingHAL::writePwmMicros(int logical_channel, float pulse_us) {
    return pwm_driver_.writeMicros(logical_channel, pulse_us);
}

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
void SpeedyBeeF405WingHAL::setSensorFrame(const SensorFrame& frame) {
    sensor_frame_ = frame;

    ImuSample imu;
    imu.gx_rad_s = frame.p_rad_s;
    imu.gy_rad_s = frame.q_rad_s;
    imu.gz_rad_s = frame.r_rad_s;
    imu.roll_rad = frame.roll_rad;
    imu.pitch_rad = frame.pitch_rad;
    imu.yaw_rad = frame.yaw_rad;
    imu_driver_.setSample(imu);

    baro_driver_.setAltitudeMeters(frame.altitude_m);

    const float rho = std::max(0.5f, config_.air_density_kg_m3);
    const float dp = 0.5f * rho * frame.pitot_airspeed_mps * frame.pitot_airspeed_mps + config_.pitot_zero_offset_pa;
    pitot_driver_.setDifferentialPressurePa(dp);
}

void SpeedyBeeF405WingHAL::setRcChannels(const std::array<float, 8>& channels_norm) {
    rc_channels_ = channels_norm;

    std::array<int, 8> pulses{};
    for (std::size_t i = 0; i < channels_norm.size(); ++i) {
        if (static_cast<int>(i) == config_.rc_throttle_channel) {
            const float t = clamp(channels_norm[i], 0.0f, 1.0f);
            pulses[i] = static_cast<int>(config_.rc_min_us + t * (config_.rc_max_us - config_.rc_min_us));
        } else {
            const float s = clamp(channels_norm[i], -1.0f, 1.0f);
            pulses[i] = static_cast<int>(config_.rc_mid_us + s * (config_.rc_max_us - config_.rc_mid_us));
        }
    }
    receiver_driver_.setPulsesUs(pulses);
}
#endif

float SpeedyBeeF405WingHAL::clamp(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}

float SpeedyBeeF405WingHAL::applyDeadband(float x) const {
    const float d = clamp(config_.rc_deadband, 0.0f, 0.2f);
    if (std::fabs(x) <= d) {
        return 0.0f;
    }
    return x;
}

float SpeedyBeeF405WingHAL::normalizeSymmetricUs(int pulse_us) const {
    const float den = std::max(1.0f, static_cast<float>(config_.rc_max_us - config_.rc_mid_us));
    const float x = static_cast<float>(pulse_us - config_.rc_mid_us) / den;
    return clamp(applyDeadband(x), -1.0f, 1.0f);
}

float SpeedyBeeF405WingHAL::normalizeThrottleUs(int pulse_us) const {
    const float den = std::max(1.0f, static_cast<float>(config_.rc_max_us - config_.rc_min_us));
    const float x = static_cast<float>(pulse_us - config_.rc_min_us) / den;
    return clamp(x, 0.0f, 1.0f);
}

float SpeedyBeeF405WingHAL::toServoPulseUs(float cmd_symm, bool reverse) const {
    float v = clamp(cmd_symm, -1.0f, 1.0f);
    if (reverse) {
        v = -v;
    }

    const float span = 0.5f * (config_.servo_max_us - config_.servo_min_us);
    return config_.servo_neutral_us + span * v;
}

float SpeedyBeeF405WingHAL::toThrottlePulseUs(float cmd_norm, bool reverse) const {
    float v = clamp(cmd_norm, 0.0f, 1.0f);
    if (reverse) {
        v = 1.0f - v;
    }
    return config_.throttle_min_us + v * (config_.throttle_max_us - config_.throttle_min_us);
}
