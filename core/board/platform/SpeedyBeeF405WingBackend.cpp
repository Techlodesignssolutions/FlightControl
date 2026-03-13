#include "SpeedyBeeF405WingBackend.h"

#include "SpeedyBeeF405WingPins.h"

#include <cstddef>

namespace {

// Low-level board hooks. Firmware can override these with real STM32 HAL/LL implementations.
extern "C" bool __attribute__((weak)) speedybee_hw_init_spi1() { return false; }
extern "C" bool __attribute__((weak)) speedybee_hw_init_i2c1() { return false; }
extern "C" bool __attribute__((weak)) speedybee_hw_init_uart1() { return false; }
extern "C" bool __attribute__((weak)) speedybee_hw_init_uart2_sbus_inverted() { return false; }
extern "C" bool __attribute__((weak)) speedybee_hw_init_adc1_ch15() { return false; }
extern "C" bool __attribute__((weak)) speedybee_hw_init_pwm(int timer_id, int channel) {
    (void)timer_id;
    (void)channel;
    return false;
}

extern "C" bool __attribute__((weak)) speedybee_hw_configure_imu() { return false; }
extern "C" bool __attribute__((weak)) speedybee_hw_configure_baro() { return false; }
extern "C" bool __attribute__((weak)) speedybee_hw_configure_pitot() { return false; }
extern "C" bool __attribute__((weak)) speedybee_hw_configure_receiver(int mode) {
    (void)mode;
    return false;
}

extern "C" bool __attribute__((weak)) speedybee_hw_read_imu(float* gx,
                                                              float* gy,
                                                              float* gz,
                                                              float* ax,
                                                              float* ay,
                                                              float* az) {
    (void)gx;
    (void)gy;
    (void)gz;
    (void)ax;
    (void)ay;
    (void)az;
    return false;
}
extern "C" bool __attribute__((weak)) speedybee_hw_read_baro_altitude(float* altitude_m) {
    (void)altitude_m;
    return false;
}
extern "C" bool __attribute__((weak)) speedybee_hw_read_pitot_dp_pa(float* dp_pa) {
    (void)dp_pa;
    return false;
}
extern "C" bool __attribute__((weak)) speedybee_hw_read_receiver_us(int* ch, std::size_t len, int mode) {
    (void)ch;
    (void)len;
    (void)mode;
    return false;
}
extern "C" bool __attribute__((weak)) speedybee_hw_write_pwm_us(int logical_channel, float pulse_us) {
    (void)logical_channel;
    (void)pulse_us;
    return false;
}

constexpr int kReceiverModeCrsf = 1;
constexpr int kReceiverModeSbus = 2;

constexpr std::array<int, 4> kPwmTimers{{
    speedybee_f405_wing::PWM0_TIMER,
    speedybee_f405_wing::PWM1_TIMER,
    speedybee_f405_wing::PWM2_TIMER,
    speedybee_f405_wing::PWM3_TIMER,
}};
constexpr std::array<int, 4> kPwmChannels{{
    speedybee_f405_wing::PWM0_TIM_CHANNEL,
    speedybee_f405_wing::PWM1_TIM_CHANNEL,
    speedybee_f405_wing::PWM2_TIM_CHANNEL,
    speedybee_f405_wing::PWM3_TIM_CHANNEL,
}};

}  // namespace

float SpeedyBeeF405WingBackend::clamp(float x, float lo, float hi) {
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

bool SpeedyBeeF405WingBackend::initSpiBus(int bus_id) {
    if (bus_id != speedybee_f405_wing::IMU_SPI_BUS) {
        return false;
    }
    if (spi1_ready_) {
        return true;
    }
    spi1_ready_ = speedybee_hw_init_spi1();
    return spi1_ready_;
}

bool SpeedyBeeF405WingBackend::initI2cBus(int bus_id) {
    if (bus_id != speedybee_f405_wing::I2C_BUS) {
        return false;
    }
    if (i2c1_ready_) {
        return true;
    }
    i2c1_ready_ = speedybee_hw_init_i2c1();
    return i2c1_ready_;
}

bool SpeedyBeeF405WingBackend::initUart(int uart_id, bool inverted_rx) {
    if (uart_id == speedybee_f405_wing::CRSF_UART && !inverted_rx) {
        if (uart1_ready_) {
            return true;
        }
        uart1_ready_ = speedybee_hw_init_uart1();
        if (uart1_ready_) {
            rx_mode_ = ReceiverMode::CRSF;
            receiver_configured_ = false;
        }
        return uart1_ready_;
    }
    if (uart_id == speedybee_f405_wing::SBUS_UART && inverted_rx) {
        if (uart2_ready_) {
            return true;
        }
        uart2_ready_ = speedybee_hw_init_uart2_sbus_inverted();
        if (uart2_ready_) {
            rx_mode_ = ReceiverMode::SBUS;
            receiver_configured_ = false;
        }
        return uart2_ready_;
    }
    return false;
}

bool SpeedyBeeF405WingBackend::initAdcChannel(int adc_id, int channel) {
    if (adc_id != speedybee_f405_wing::AIRSPEED_ADC ||
        channel != speedybee_f405_wing::AIRSPEED_ADC_CHANNEL) {
        return false;
    }
    if (adc1_ch15_ready_) {
        return true;
    }
    adc1_ch15_ready_ = speedybee_hw_init_adc1_ch15();
    return adc1_ch15_ready_;
}

bool SpeedyBeeF405WingBackend::initPwmTimerChannel(int timer_id, int channel) {
    for (std::size_t i = 0; i < kPwmTimers.size(); ++i) {
        if (kPwmTimers[i] == timer_id && kPwmChannels[i] == channel) {
            if (pwm_ready_[i]) {
                return true;
            }
            pwm_ready_[i] = speedybee_hw_init_pwm(timer_id, channel);
            return pwm_ready_[i];
        }
    }
    return false;
}

bool SpeedyBeeF405WingBackend::readImuRaw(Stm32f4Platform::ImuRaw& out) {
    if (!spi1_ready_) {
        return false;
    }
    if (!imu_configured_) {
        if (!speedybee_hw_configure_imu()) {
            return false;
        }
        imu_configured_ = true;
    }

    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    if (!speedybee_hw_read_imu(&gx, &gy, &gz, &ax, &ay, &az)) {
        return false;
    }

    out.gx_rad_s = gx;
    out.gy_rad_s = gy;
    out.gz_rad_s = gz;
    out.ax_m_s2 = ax;
    out.ay_m_s2 = ay;
    out.az_m_s2 = az;
    return true;
}

bool SpeedyBeeF405WingBackend::readBaroAltitudeMeters(float& altitude_m) {
    if (!i2c1_ready_) {
        return false;
    }
    if (!baro_configured_) {
        if (!speedybee_hw_configure_baro()) {
            return false;
        }
        baro_configured_ = true;
    }
    return speedybee_hw_read_baro_altitude(&altitude_m);
}

bool SpeedyBeeF405WingBackend::readPitotDifferentialPressurePa(float& dp_pa) {
    if (!pitot_configured_) {
        if (!(i2c1_ready_ || adc1_ch15_ready_)) {
            return false;
        }
        if (!speedybee_hw_configure_pitot()) {
            return false;
        }
        pitot_configured_ = true;
    }
    return speedybee_hw_read_pitot_dp_pa(&dp_pa);
}

bool SpeedyBeeF405WingBackend::readReceiverPulsesUs(std::array<int, 8>& out) {
    if ((rx_mode_ == ReceiverMode::CRSF && !uart1_ready_) ||
        (rx_mode_ == ReceiverMode::SBUS && !uart2_ready_) ||
        rx_mode_ == ReceiverMode::None) {
        return false;
    }

    const int mode = (rx_mode_ == ReceiverMode::CRSF) ? kReceiverModeCrsf : kReceiverModeSbus;
    if (!receiver_configured_) {
        if (!speedybee_hw_configure_receiver(mode)) {
            return false;
        }
        receiver_configured_ = true;
    }

    std::array<int, 8> pulses{};
    if (!speedybee_hw_read_receiver_us(pulses.data(), pulses.size(), mode)) {
        return false;
    }

    out = pulses;
    return true;
}

bool SpeedyBeeF405WingBackend::writePwmMicros(int logical_channel, float pulse_us) {
    if (logical_channel < 0 || logical_channel >= static_cast<int>(pwm_ready_.size())) {
        return false;
    }
    if (!pwm_ready_[static_cast<std::size_t>(logical_channel)]) {
        return false;
    }

    const float clamped = clamp(pulse_us, 800.0f, 2200.0f);
    if (!speedybee_hw_write_pwm_us(logical_channel, clamped)) {
        return false;
    }

    last_pwm_us_[static_cast<std::size_t>(logical_channel)] = clamped;
    return true;
}
