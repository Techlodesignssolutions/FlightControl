#pragma once

#include <array>

class ReceiverDriver {
public:
    bool init();
    bool readPulsesUs(std::array<int, 8>& pulses_us) const;

    void setPulsesUs(const std::array<int, 8>& pulses_us);

private:
    std::array<int, 8> pulses_us_{{1500,1500,1500,1000,1500,1500,1500,1500}};
    bool initialized_{false};
};
