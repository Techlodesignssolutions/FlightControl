#pragma once

#include "../platform/Stm32f4Platform.h"

class BaroDriver {
public:
    bool init();
    bool readAltitudeMeters(float& altitude_m) const;

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    void setAltitudeMeters(float altitude_m);
#endif

private:
    Stm32f4Platform platform_{};
    bool initialized_{false};
};
