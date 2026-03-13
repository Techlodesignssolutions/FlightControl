#include "Stm32f4Platform.h"

Stm32f4Platform::Backend* Stm32f4Platform::backend_ = nullptr;
Stm32f4Platform::ImuRaw Stm32f4Platform::imu_raw_{};
float Stm32f4Platform::baro_altitude_m_ = 0.0f;
float Stm32f4Platform::pitot_dp_pa_ = 0.0f;
std::array<int, 8> Stm32f4Platform::receiver_pulses_us_{{1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500}};
std::array<float, 8> Stm32f4Platform::pwm_pulses_us_{{1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500}};

void Stm32f4Platform::installBackend(Backend* backend) {
    backend_ = backend;
}

Stm32f4Platform::Backend* Stm32f4Platform::backend() {
    return backend_;
}

bool Stm32f4Platform::initSpiBus(int bus_id) const {
    if (backend_ != nullptr) {
        return backend_->initSpiBus(bus_id);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    (void)bus_id;
    return true;
#else
    (void)bus_id;
    return false;
#endif
}

bool Stm32f4Platform::initI2cBus(int bus_id) const {
    if (backend_ != nullptr) {
        return backend_->initI2cBus(bus_id);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    (void)bus_id;
    return true;
#else
    (void)bus_id;
    return false;
#endif
}

bool Stm32f4Platform::initUart(int uart_id, bool inverted_rx) const {
    if (backend_ != nullptr) {
        return backend_->initUart(uart_id, inverted_rx);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    (void)uart_id;
    (void)inverted_rx;
    return true;
#else
    (void)uart_id;
    (void)inverted_rx;
    return false;
#endif
}

bool Stm32f4Platform::initAdcChannel(int adc_id, int channel) const {
    if (backend_ != nullptr) {
        return backend_->initAdcChannel(adc_id, channel);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    (void)adc_id;
    (void)channel;
    return true;
#else
    (void)adc_id;
    (void)channel;
    return false;
#endif
}

bool Stm32f4Platform::initPwmTimerChannel(int timer_id, int channel) const {
    if (backend_ != nullptr) {
        return backend_->initPwmTimerChannel(timer_id, channel);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    (void)timer_id;
    (void)channel;
    return true;
#else
    (void)timer_id;
    (void)channel;
    return false;
#endif
}

bool Stm32f4Platform::readImuRaw(ImuRaw& out) const {
    if (backend_ != nullptr) {
        return backend_->readImuRaw(out);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    out = imu_raw_;
    return true;
#else
    (void)out;
    return false;
#endif
}

bool Stm32f4Platform::readBaroAltitudeMeters(float& altitude_m) const {
    if (backend_ != nullptr) {
        return backend_->readBaroAltitudeMeters(altitude_m);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    altitude_m = baro_altitude_m_;
    return true;
#else
    (void)altitude_m;
    return false;
#endif
}

bool Stm32f4Platform::readPitotDifferentialPressurePa(float& dp_pa) const {
    if (backend_ != nullptr) {
        return backend_->readPitotDifferentialPressurePa(dp_pa);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    dp_pa = pitot_dp_pa_;
    return true;
#else
    (void)dp_pa;
    return false;
#endif
}

bool Stm32f4Platform::readReceiverPulsesUs(std::array<int, 8>& out) const {
    if (backend_ != nullptr) {
        return backend_->readReceiverPulsesUs(out);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    out = receiver_pulses_us_;
    return true;
#else
    (void)out;
    return false;
#endif
}

bool Stm32f4Platform::writePwmMicros(int logical_channel, float pulse_us) const {
    if (backend_ != nullptr) {
        return backend_->writePwmMicros(logical_channel, pulse_us);
    }
#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    if (logical_channel < 0 || logical_channel >= static_cast<int>(pwm_pulses_us_.size())) {
        return false;
    }
    pwm_pulses_us_[static_cast<std::size_t>(logical_channel)] = pulse_us;
    return true;
#else
    (void)logical_channel;
    (void)pulse_us;
    return false;
#endif
}

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
void Stm32f4Platform::injectImuRaw(const ImuRaw& raw) const {
    imu_raw_ = raw;
}

void Stm32f4Platform::injectBaroAltitudeMeters(float altitude_m) const {
    baro_altitude_m_ = altitude_m;
}

void Stm32f4Platform::injectPitotDifferentialPressurePa(float dp_pa) const {
    pitot_dp_pa_ = dp_pa;
}

void Stm32f4Platform::injectReceiverPulsesUs(const std::array<int, 8>& pulses) const {
    receiver_pulses_us_ = pulses;
}
#endif
