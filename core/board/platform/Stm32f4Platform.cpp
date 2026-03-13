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
#include "BenchBackend.h"

namespace {
BenchBackend g_bench_backend{};

BenchBackend* ensureBenchBackend() {
    if (Stm32f4Platform::backend() == nullptr) {
        Stm32f4Platform::installBackend(&g_bench_backend);
        return &g_bench_backend;
    }
    return dynamic_cast<BenchBackend*>(Stm32f4Platform::backend());
}
}  // namespace

void Stm32f4Platform::injectImuRaw(const ImuRaw& raw) const {
    if (BenchBackend* bench = ensureBenchBackend()) {
        bench->setImuRaw(raw);
    }
}

void Stm32f4Platform::injectBaroAltitudeMeters(float altitude_m) const {
    if (BenchBackend* bench = ensureBenchBackend()) {
        bench->setBaroAltitudeMeters(altitude_m);
    }
}

void Stm32f4Platform::injectPitotDifferentialPressurePa(float dp_pa) const {
    if (BenchBackend* bench = ensureBenchBackend()) {
        bench->setPitotDifferentialPressurePa(dp_pa);
    }
}

void Stm32f4Platform::injectReceiverPulsesUs(const std::array<int, 8>& pulses) const {
    if (BenchBackend* bench = ensureBenchBackend()) {
        bench->setReceiverPulsesUs(pulses);
    }
}
#endif
