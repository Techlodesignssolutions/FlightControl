#pragma once

#include <array>

class ReceiverDriver {
public:
    enum class Protocol {
        CRSF,
        SBUS
    };

    bool init(Protocol protocol);
    bool readPulsesUs(std::array<int, 8>& pulses_us) const;

    void setPulsesUs(const std::array<int, 8>& pulses_us);

private:
    Protocol protocol_{Protocol::CRSF};
    std::array<int, 8> pulses_us_{{1500,1500,1500,1000,1500,1500,1500,1500}};
    bool initialized_{false};
};
