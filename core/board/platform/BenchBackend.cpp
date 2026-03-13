#include "BenchBackend.h"

#include <cstddef>

bool BenchBackend::initSpiBus(int) { return true; }
bool BenchBackend::initI2cBus(int) { return true; }
bool BenchBackend::initUart(int, bool) { return true; }
bool BenchBackend::initAdcChannel(int, int) { return true; }
bool BenchBackend::initPwmTimerChannel(int, int) { return true; }

bool BenchBackend::readImuRaw(Stm32f4Platform::ImuRaw& out) {
    out = imu_raw_;
    return true;
}

bool BenchBackend::readBaroAltitudeMeters(float& altitude_m) {
    altitude_m = baro_altitude_m_;
    return true;
}

bool BenchBackend::readPitotDifferentialPressurePa(float& dp_pa) {
    dp_pa = pitot_dp_pa_;
    return true;
}

bool BenchBackend::readReceiverPulsesUs(std::array<int, 8>& out) {
    out = receiver_pulses_us_;
    return true;
}

bool BenchBackend::writePwmMicros(int logical_channel, float pulse_us) {
    if (logical_channel < 0 || logical_channel >= static_cast<int>(pwm_pulses_us_.size())) {
        return false;
    }
    pwm_pulses_us_[static_cast<std::size_t>(logical_channel)] = pulse_us;
    return true;
}

void BenchBackend::setImuRaw(const Stm32f4Platform::ImuRaw& raw) {
    imu_raw_ = raw;
}

void BenchBackend::setBaroAltitudeMeters(float altitude_m) {
    baro_altitude_m_ = altitude_m;
}

void BenchBackend::setPitotDifferentialPressurePa(float dp_pa) {
    pitot_dp_pa_ = dp_pa;
}

void BenchBackend::setReceiverPulsesUs(const std::array<int, 8>& pulses) {
    receiver_pulses_us_ = pulses;
}
