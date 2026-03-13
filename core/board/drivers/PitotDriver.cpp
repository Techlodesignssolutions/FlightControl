#include "PitotDriver.h"

// NOTE: Thin driver wrapper; chip/peripheral-specific implementation lives in board backend.

#include "../platform/SpeedyBeeF405WingPins.h"

bool PitotDriver::init(Source source) {
    source_ = source;
    if (source_ == Source::DigitalI2C) {
        initialized_ = platform_.initI2cBus(speedybee_f405_wing::I2C_BUS);
    } else {
        initialized_ = platform_.initAdcChannel(speedybee_f405_wing::AIRSPEED_ADC,
                                                speedybee_f405_wing::AIRSPEED_ADC_CHANNEL);
    }
    return initialized_;
}

bool PitotDriver::readDifferentialPressurePa(float& dp_pa) const {
    if (!initialized_) {
        return false;
    }
    return platform_.readPitotDifferentialPressurePa(dp_pa);
}

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
void PitotDriver::setDifferentialPressurePa(float dp_pa) {
    platform_.injectPitotDifferentialPressurePa(dp_pa);
}
#endif
