#include "ReceiverDriver.h"

// NOTE: Stub transport implementation for bring-up; replace with real board I/O backend.

#include "../platform/SpeedyBeeF405WingPins.h"

bool ReceiverDriver::init(Protocol protocol) {
    protocol_ = protocol;
    if (protocol_ == Protocol::CRSF) {
        initialized_ = platform_.initUart(speedybee_f405_wing::CRSF_UART, false);
    } else {
        initialized_ = platform_.initUart(speedybee_f405_wing::SBUS_UART, true);
    }
    return initialized_;
}

bool ReceiverDriver::readPulsesUs(std::array<int, 8>& pulses_us) const {
    if (!initialized_) {
        return false;
    }
    return platform_.readReceiverPulsesUs(pulses_us);
}

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
void ReceiverDriver::setPulsesUs(const std::array<int, 8>& pulses_us) {
    platform_.injectReceiverPulsesUs(pulses_us);
}
#endif
