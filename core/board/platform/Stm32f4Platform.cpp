#include "Stm32f4Platform.h"

Stm32f4Platform::ImuRaw Stm32f4Platform::imu_raw_{};
float Stm32f4Platform::baro_altitude_m_ = 0.0f;
float Stm32f4Platform::pitot_dp_pa_ = 0.0f;
std::array<int, 8> Stm32f4Platform::receiver_pulses_us_{{1500,1500,1500,1000,1500,1500,1500,1500}};
std::array<float, 8> Stm32f4Platform::pwm_pulses_us_{{1500,1500,1500,1000,1500,1500,1500,1500}};

bool Stm32f4Platform::initSpiBus(int) const { return true; }
bool Stm32f4Platform::initI2cBus(int) const { return true; }
bool Stm32f4Platform::initUart(int, bool) const { return true; }
bool Stm32f4Platform::initAdcChannel(int, int) const { return true; }
bool Stm32f4Platform::initPwmTimerChannel(int, int) const { return true; }

bool Stm32f4Platform::readImuRaw(ImuRaw& out) const { out = imu_raw_; return true; }
bool Stm32f4Platform::readBaroAltitudeMeters(float& altitude_m) const { altitude_m = baro_altitude_m_; return true; }
bool Stm32f4Platform::readPitotDifferentialPressurePa(float& dp_pa) const { dp_pa = pitot_dp_pa_; return true; }
bool Stm32f4Platform::readReceiverPulsesUs(std::array<int, 8>& out) const { out = receiver_pulses_us_; return true; }

bool Stm32f4Platform::writePwmMicros(int logical_channel, float pulse_us) const {
    if (logical_channel < 0 || logical_channel >= static_cast<int>(pwm_pulses_us_.size())) {
        return false;
    }
    pwm_pulses_us_[static_cast<std::size_t>(logical_channel)] = pulse_us;
    return true;
}

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
void Stm32f4Platform::injectImuRaw(const ImuRaw& raw) const { imu_raw_ = raw; }
void Stm32f4Platform::injectBaroAltitudeMeters(float altitude_m) const { baro_altitude_m_ = altitude_m; }
void Stm32f4Platform::injectPitotDifferentialPressurePa(float dp_pa) const { pitot_dp_pa_ = dp_pa; }
void Stm32f4Platform::injectReceiverPulsesUs(const std::array<int, 8>& pulses) const { receiver_pulses_us_ = pulses; }
#endif
