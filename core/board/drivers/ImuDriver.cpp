#include "ImuDriver.h"

// NOTE: Stub transport implementation for bring-up; replace with real board I/O backend.

#include "../platform/SpeedyBeeF405WingPins.h"

bool ImuDriver::init() {
    initialized_ = platform_.initSpiBus(speedybee_f405_wing::IMU_SPI_BUS);
    return initialized_;
}

bool ImuDriver::read(ImuSample& out) const {
    if (!initialized_) {
        return false;
    }
    out = sample_;
    return true;
}

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
void ImuDriver::setSample(const ImuSample& sample) {
    sample_ = sample;
}
#endif
