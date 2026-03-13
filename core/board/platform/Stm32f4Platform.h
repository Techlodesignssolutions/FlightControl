#pragma once

#include <array>

class Stm32f4Platform {
public:
    struct ImuRaw {
        float gx_rad_s = 0.0f;
        float gy_rad_s = 0.0f;
        float gz_rad_s = 0.0f;
        float ax_m_s2 = 0.0f;
        float ay_m_s2 = 0.0f;
        float az_m_s2 = 9.80665f;
    };

    bool initSpiBus(int bus_id) const;
    bool initI2cBus(int bus_id) const;
    bool initUart(int uart_id, bool inverted_rx) const;
    bool initAdcChannel(int adc_id, int channel) const;
    bool initPwmTimerChannel(int timer_id, int channel) const;

    bool readImuRaw(ImuRaw& out) const;
    bool readBaroAltitudeMeters(float& altitude_m) const;
    bool readPitotDifferentialPressurePa(float& dp_pa) const;
    bool readReceiverPulsesUs(std::array<int, 8>& out) const;
    bool writePwmMicros(int logical_channel, float pulse_us) const;

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
    void injectImuRaw(const ImuRaw& raw) const;
    void injectBaroAltitudeMeters(float altitude_m) const;
    void injectPitotDifferentialPressurePa(float dp_pa) const;
    void injectReceiverPulsesUs(const std::array<int, 8>& pulses) const;
#endif

private:
    // Shared stub transport state until real STM32 HAL/LL backend is wired.
    static ImuRaw imu_raw_;
    static float baro_altitude_m_;
    static float pitot_dp_pa_;
    static std::array<int, 8> receiver_pulses_us_;
    static std::array<float, 8> pwm_pulses_us_;
};
