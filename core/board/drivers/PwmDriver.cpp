#include "PwmDriver.h"

bool PwmDriver::init() {
    initialized_ = true;
    return true;
}

bool PwmDriver::writeMicros(int logical_channel, float pulse_us) {
    if (!initialized_ || logical_channel < 0 || logical_channel >= static_cast<int>(last_written_us_.size())) {
        return false;
    }
    last_written_us_[static_cast<std::size_t>(logical_channel)] = pulse_us;
    return true;
}
