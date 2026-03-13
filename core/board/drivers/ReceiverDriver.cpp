#include "ReceiverDriver.h"

bool ReceiverDriver::init() {
    initialized_ = true;
    return true;
}

bool ReceiverDriver::readPulsesUs(std::array<int, 8>& pulses_us) const {
    if (!initialized_) {
        return false;
    }
    pulses_us = pulses_us_;
    return true;
}

void ReceiverDriver::setPulsesUs(const std::array<int, 8>& pulses_us) {
    pulses_us_ = pulses_us;
}
