#include "Stm32f4Platform.h"

bool Stm32f4Platform::initSpiBus(int) const {
    return true;
}

bool Stm32f4Platform::initI2cBus(int) const {
    return true;
}

bool Stm32f4Platform::initUart(int, bool) const {
    return true;
}

bool Stm32f4Platform::initAdcChannel(int, int) const {
    return true;
}

bool Stm32f4Platform::initPwmTimerChannel(int, int) const {
    return true;
}
