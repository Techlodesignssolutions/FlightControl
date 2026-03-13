#include "PwmDriver.h"

#include "../platform/SpeedyBeeF405WingPins.h"

bool PwmDriver::init() {
    bool ok = true;
    ok = ok && platform_.initPwmTimerChannel(speedybee_f405_wing::PWM0_TIMER, speedybee_f405_wing::PWM0_TIM_CHANNEL);
    ok = ok && platform_.initPwmTimerChannel(speedybee_f405_wing::PWM1_TIMER, speedybee_f405_wing::PWM1_TIM_CHANNEL);
    ok = ok && platform_.initPwmTimerChannel(speedybee_f405_wing::PWM2_TIMER, speedybee_f405_wing::PWM2_TIM_CHANNEL);
    ok = ok && platform_.initPwmTimerChannel(speedybee_f405_wing::PWM3_TIMER, speedybee_f405_wing::PWM3_TIM_CHANNEL);
    initialized_ = ok;
    return initialized_;
}

bool PwmDriver::writeMicros(int logical_channel, float pulse_us) {
    if (!initialized_ || logical_channel < 0 || logical_channel >= static_cast<int>(last_written_us_.size())) {
        return false;
    }
    last_written_us_[static_cast<std::size_t>(logical_channel)] = pulse_us;
    return true;
}
