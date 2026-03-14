#pragma once

#include "Stm32f4Platform.h"

#include <array>

class BenchBackend final : public Stm32f4Platform::Backend {
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

    void setImuRaw(const Stm32f4Platform::ImuRaw& raw);
    void setBaroAltitudeMeters(float altitude_m);
    void setPitotDifferentialPressurePa(float dp_pa);
    void setReceiverPulsesUs(const std::array<int, 8>& pulses);

private:
    Stm32f4Platform::ImuRaw imu_raw_{};
    float baro_altitude_m_{0.0f};
    float pitot_dp_pa_{0.0f};
    std::array<int, 8> receiver_pulses_us_{{1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500}};
    std::array<float, 8> pwm_pulses_us_{{1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500}};
};
