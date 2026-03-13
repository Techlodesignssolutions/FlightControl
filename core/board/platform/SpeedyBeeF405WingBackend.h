#pragma once

#include "Stm32f4Platform.h"

#include <array>

class SpeedyBeeF405WingBackend final : public Stm32f4Platform::Backend {
public:
    bool initSpiBus(int bus_id) override;
    bool initI2cBus(int bus_id) override;
    bool initUart(int uart_id, bool inverted_rx) override;
    bool initAdcChannel(int adc_id, int channel) override;
    bool initPwmTimerChannel(int timer_id, int channel) override;

    bool readImuRaw(Stm32f4Platform::ImuRaw& out) override;
    bool readBaroAltitudeMeters(float& altitude_m) override;
    bool readPitotDifferentialPressurePa(float& dp_pa) override;
    bool readReceiverPulsesUs(std::array<int, 8>& out) override;
    bool writePwmMicros(int logical_channel, float pulse_us) override;

private:
    enum class ReceiverMode {
        None,
        CRSF,
        SBUS
    };

    static float clamp(float x, float lo, float hi);

    bool spi1_ready_{false};
    bool i2c1_ready_{false};
    bool uart1_ready_{false};
    bool uart2_ready_{false};
    bool adc1_ch15_ready_{false};

    std::array<bool, 4> pwm_ready_{{false, false, false, false}};
    std::array<float, 8> last_pwm_us_{{1500.0f, 1500.0f, 1500.0f, 1000.0f, 1500.0f, 1500.0f, 1500.0f, 1500.0f}};

    bool imu_configured_{false};
    bool baro_configured_{false};
    bool pitot_configured_{false};
    bool receiver_configured_{false};

    ReceiverMode rx_mode_{ReceiverMode::None};
};
