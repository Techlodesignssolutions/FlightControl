#include "PitotDriver.h"

bool PitotDriver::init(Source source) {
    source_ = source;
    initialized_ = true;
    return true;
}

bool PitotDriver::readDifferentialPressurePa(float& dp_pa) const {
    if (!initialized_) {
        return false;
    }
    dp_pa = dp_pa_;
    return true;
}

void PitotDriver::setDifferentialPressurePa(float dp_pa) {
    dp_pa_ = dp_pa;
}
