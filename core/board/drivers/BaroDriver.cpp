#include "BaroDriver.h"

bool BaroDriver::init() {
    initialized_ = true;
    return true;
}

bool BaroDriver::readAltitudeMeters(float& altitude_m) const {
    if (!initialized_) {
        return false;
    }
    altitude_m = altitude_m_;
    return true;
}

void BaroDriver::setAltitudeMeters(float altitude_m) {
    altitude_m_ = altitude_m;
}
