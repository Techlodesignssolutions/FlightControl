#pragma once

#include "../platform/Stm32f4Platform.h"

class PitotDriver {
public:
    enum class Source {
        DigitalI2C,
        AnalogAir
    };

    bool init(Source source);
    bool readDifferentialPressurePa(float& dp_pa) const;

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    void setDifferentialPressurePa(float dp_pa);
#endif

private:
    Stm32f4Platform platform_{};
    Source source_{Source::DigitalI2C};
    bool initialized_{false};
};
