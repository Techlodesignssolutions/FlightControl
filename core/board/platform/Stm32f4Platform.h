#pragma once

class Stm32f4Platform {
public:
    bool initSpiBus(int bus_id) const;
    bool initI2cBus(int bus_id) const;
    bool initUart(int uart_id, bool inverted_rx) const;
    bool initAdcChannel(int adc_id, int channel) const;
    bool initPwmTimerChannel(int timer_id, int channel) const;
};
