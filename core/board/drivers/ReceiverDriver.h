#pragma once

#include "../platform/Stm32f4Platform.h"

#include <array>

class ReceiverDriver {
public:
    enum class Protocol {
        CRSF,
        SBUS
    };

    bool init(Protocol protocol);
    bool readPulsesUs(std::array<int, 8>& pulses_us) const;

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    void setPulsesUs(const std::array<int, 8>& pulses_us);
#endif

private:
    Stm32f4Platform platform_{};
    Protocol protocol_{Protocol::CRSF};
    bool initialized_{false};
};
