#pragma once

#include <array>

class PwmDriver {
public:
    bool init();
    bool writeMicros(int logical_channel, float pulse_us);

    const std::array<float, 8>& lastWrittenMicros() const { return last_written_us_; }

private:
    std::array<float, 8> last_written_us_{{1500,1500,1500,1000,1500,1500,1500,1500}};
    bool initialized_{false};
};
