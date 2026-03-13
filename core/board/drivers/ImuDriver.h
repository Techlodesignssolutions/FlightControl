#pragma once

#include "../platform/Stm32f4Platform.h"

struct ImuSample {
    float gx_rad_s = 0.0f;
    float gy_rad_s = 0.0f;
    float gz_rad_s = 0.0f;

    float ax_m_s2 = 0.0f;
    float ay_m_s2 = 0.0f;
    float az_m_s2 = 9.80665f;
};

class ImuDriver {
public:
    bool init();
    bool read(ImuSample& out) const;

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    void setSample(const ImuSample& sample);
#endif

private:
    Stm32f4Platform platform_{};
    ImuSample sample_{};
    bool initialized_{false};
};
