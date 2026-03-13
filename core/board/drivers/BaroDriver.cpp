#include "BaroDriver.h"

// NOTE: Stub transport implementation for bring-up; replace with real board I/O backend.

#include "../platform/SpeedyBeeF405WingPins.h"

bool BaroDriver::init() {
    initialized_ = platform_.initI2cBus(speedybee_f405_wing::I2C_BUS);
    return initialized_;
}

bool BaroDriver::readAltitudeMeters(float& altitude_m) const {
    if (!initialized_) {
        return false;
    }
    altitude_m = altitude_m_;
    return true;
}

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
void BaroDriver::setAltitudeMeters(float altitude_m) {
    altitude_m_ = altitude_m;
}
#endif
