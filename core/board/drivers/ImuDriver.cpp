#include "ImuDriver.h"

bool ImuDriver::init() {
    initialized_ = true;
    return true;
}

bool ImuDriver::read(ImuSample& out) const {
    if (!initialized_) {
        return false;
    }
    out = sample_;
    return true;
}

void ImuDriver::setSample(const ImuSample& sample) {
    sample_ = sample;
}
