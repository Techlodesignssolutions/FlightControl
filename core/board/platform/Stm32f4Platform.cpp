#include "Stm32f4Platform.h"

Stm32f4Platform::Backend* Stm32f4Platform::backend_ = nullptr;

void Stm32f4Platform::installBackend(Backend* backend) {
    backend_ = backend;
}

Stm32f4Platform::Backend* Stm32f4Platform::backend() {
    return backend_;
}

bool Stm32f4Platform::initSpiBus(int bus_id) const {
    return backend_ != nullptr && backend_->initSpiBus(bus_id);
}

bool Stm32f4Platform::initI2cBus(int bus_id) const {
    return backend_ != nullptr && backend_->initI2cBus(bus_id);
}

bool Stm32f4Platform::initUart(int uart_id, bool inverted_rx) const {
    return backend_ != nullptr && backend_->initUart(uart_id, inverted_rx);
}

bool Stm32f4Platform::initAdcChannel(int adc_id, int channel) const {
    return backend_ != nullptr && backend_->initAdcChannel(adc_id, channel);
}

bool Stm32f4Platform::initPwmTimerChannel(int timer_id, int channel) const {
    return backend_ != nullptr && backend_->initPwmTimerChannel(timer_id, channel);
}

bool Stm32f4Platform::readImuRaw(ImuRaw& out) const {
    return backend_ != nullptr && backend_->readImuRaw(out);
}

bool Stm32f4Platform::readBaroAltitudeMeters(float& altitude_m) const {
    return backend_ != nullptr && backend_->readBaroAltitudeMeters(altitude_m);
}

bool Stm32f4Platform::readPitotDifferentialPressurePa(float& dp_pa) const {
    return backend_ != nullptr && backend_->readPitotDifferentialPressurePa(dp_pa);
}

bool Stm32f4Platform::readReceiverPulsesUs(std::array<int, 8>& out) const {
    return backend_ != nullptr && backend_->readReceiverPulsesUs(out);
}

bool Stm32f4Platform::writePwmMicros(int logical_channel, float pulse_us) const {
    return backend_ != nullptr && backend_->writePwmMicros(logical_channel, pulse_us);
}

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
namespace {
class StubBackend final : public Stm32f4Platform::Backend {
public:
    bool initSpiBus(int) override { return true; }
    bool initI2cBus(int) override { return true; }
    bool initUart(int, bool) override { return true; }
    bool initAdcChannel(int, int) override { return true; }
    bool initPwmTimerChannel(int, int) override { return true; }

    bool readImuRaw(Stm32f4Platform::ImuRaw& out) override {
        out = imu_raw_;
        return true;
    }

    bool readBaroAltitudeMeters(float& altitude_m) override {
        altitude_m = baro_altitude_m_;
        return true;
    }

    bool readPitotDifferentialPressurePa(float& dp_pa) override {
        dp_pa = pitot_dp_pa_;
        return true;
    }

    bool readReceiverPulsesUs(std::array<int, 8>& out) override {
        out = receiver_pulses_us_;
        return true;
    }

    bool writePwmMicros(int logical_channel, float pulse_us) override {
        if (logical_channel < 0 || logical_channel >= static_cast<int>(pwm_pulses_us_.size())) {
            return false;
        }
        pwm_pulses_us_[static_cast<std::size_t>(logical_channel)] = pulse_us;
        return true;
    }

    void setImuRaw(const Stm32f4Platform::ImuRaw& raw) {
        imu_raw_ = raw;
    }

    void setBaroAltitudeMeters(float altitude_m) {
        baro_altitude_m_ = altitude_m;
    }

    void setPitotDifferentialPressurePa(float dp_pa) {
        pitot_dp_pa_ = dp_pa;
    }

    void setReceiverPulsesUs(const std::array<int, 8>& pulses) {
        receiver_pulses_us_ = pulses;
    }

private:
    Stm32f4Platform::ImuRaw imu_raw_{};
    float baro_altitude_m_{0.0f};
    float pitot_dp_pa_{0.0f};
    std::array<int, 8> receiver_pulses_us_{{1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500}};
    std::array<float, 8> pwm_pulses_us_{{1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500}};
};

StubBackend g_stub_backend{};

StubBackend* ensureStubBackend() {
    if (Stm32f4Platform::backend() == nullptr) {
        Stm32f4Platform::installBackend(&g_stub_backend);
        return &g_stub_backend;
    }
    return dynamic_cast<StubBackend*>(Stm32f4Platform::backend());
}
}  // namespace

void Stm32f4Platform::injectImuRaw(const ImuRaw& raw) const {
    if (StubBackend* stub = ensureStubBackend()) {
        stub->setImuRaw(raw);
    }
}

void Stm32f4Platform::injectBaroAltitudeMeters(float altitude_m) const {
    if (StubBackend* stub = ensureStubBackend()) {
        stub->setBaroAltitudeMeters(altitude_m);
    }
}

void Stm32f4Platform::injectPitotDifferentialPressurePa(float dp_pa) const {
    if (StubBackend* stub = ensureStubBackend()) {
        stub->setPitotDifferentialPressurePa(dp_pa);
    }
}

void Stm32f4Platform::injectReceiverPulsesUs(const std::array<int, 8>& pulses) const {
    if (StubBackend* stub = ensureStubBackend()) {
        stub->setReceiverPulsesUs(pulses);
    }
}
#endif
