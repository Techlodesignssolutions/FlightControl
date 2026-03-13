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

extern "C" bool __attribute__((weak)) speedybee_hw_probe_imu(std::uint8_t* who_am_i) {
    (void)who_am_i;
    return false;
}
extern "C" bool __attribute__((weak)) speedybee_hw_probe_baro(std::uint8_t i2c_addr) {
    (void)i2c_addr;
    return false;
}
extern "C" bool __attribute__((weak)) speedybee_hw_probe_pitot() { return false; }

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
extern "C" int __attribute__((weak)) speedybee_hw_read_uart_bytes(int uart_id, std::uint8_t* out, std::size_t max_len) {
    (void)uart_id;
    (void)out;
    (void)max_len;
    return 0;
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

constexpr std::size_t kCrsfMinFrameLen = 4;
constexpr std::uint8_t kCrsfTypeRcChannelsPacked = 0x16;
constexpr std::size_t kSbusFrameLen = 25;
constexpr std::uint8_t kSbusStartByte = 0x0F;

float mapRxRawToUs(std::uint16_t raw) {
    // Common CRSF/SBUS 11-bit nominal range mapping.
    constexpr float in_min = 172.0f;
    constexpr float in_max = 1811.0f;
    constexpr float out_min = 1000.0f;
    constexpr float out_max = 2000.0f;

    float x = static_cast<float>(raw);
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

std::uint8_t crsfCrc8(const std::uint8_t* data, std::size_t len) {
    std::uint8_t crc = 0;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = static_cast<std::uint8_t>((crc << 1U) ^ 0xD5U);
            } else {
                crc = static_cast<std::uint8_t>(crc << 1U);
            }
        }
    }
    return crc;
}

bool unpack11BitChannels(const std::uint8_t* packed, std::size_t packed_len, std::array<int, 8>& out_us) {
    std::uint32_t bitbuf = 0;
    int bits = 0;
    std::size_t byte_i = 0;

    for (std::size_t ch = 0; ch < out_us.size(); ++ch) {
        while (bits < 11) {
            if (byte_i >= packed_len) {
                return false;
            }
            bitbuf |= (static_cast<std::uint32_t>(packed[byte_i]) << bits);
            bits += 8;
            ++byte_i;
        }

        const std::uint16_t raw = static_cast<std::uint16_t>(bitbuf & 0x7FFU);
        bitbuf >>= 11;
        bits -= 11;
        out_us[ch] = static_cast<int>(mapRxRawToUs(raw));
    }

    return true;
}

bool tryParseCrsf(const std::uint8_t* buf,
                  std::size_t size,
                  std::size_t& consumed,
                  std::array<int, 8>& out_us) {
    consumed = 0;
    if (size < kCrsfMinFrameLen) {
        return false;
    }

    // CRSF frame: [addr][len][type][payload...][crc] where total = len + 2.
    const std::uint8_t len = buf[1];
    const std::size_t frame_len = static_cast<std::size_t>(len) + 2U;
    if (frame_len < kCrsfMinFrameLen) {
        consumed = 1;
        return false;
    }
    if (size < frame_len) {
        return false;
    }

    const std::uint8_t type = buf[2];
    const std::uint8_t crc_rx = buf[frame_len - 1U];
    const std::uint8_t crc_calc = crsfCrc8(&buf[2], frame_len - 3U);
    if (crc_rx != crc_calc) {
        consumed = 1;
        return false;
    }

    consumed = frame_len;
    if (type != kCrsfTypeRcChannelsPacked) {
        return false;
    }

    const std::uint8_t* payload = &buf[3];
    const std::size_t payload_len = frame_len - 4U;
    return unpack11BitChannels(payload, payload_len, out_us);
}

bool tryParseSbus(const std::uint8_t* buf,
                  std::size_t size,
                  std::size_t& consumed,
                  std::array<int, 8>& out_us) {
    consumed = 0;
    if (size < 1) {
        return false;
    }
    if (buf[0] != kSbusStartByte) {
        consumed = 1;
        return false;
    }
    if (size < kSbusFrameLen) {
        return false;
    }

    consumed = kSbusFrameLen;
    return unpack11BitChannels(&buf[1], 22, out_us);
}

template <std::size_t N>
void consumeBytes(std::array<std::uint8_t, N>& buffer, std::size_t& size, std::size_t consumed) {
    if (consumed == 0 || size == 0) {
        return;
    }
    if (consumed >= size) {
        size = 0;
        return;
    }
    const std::size_t remain = size - consumed;
    for (std::size_t i = 0; i < remain; ++i) {
        buffer[i] = buffer[i + consumed];
    }
    size = remain;
}

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
            rx_stream_size_ = 0;
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
            rx_stream_size_ = 0;
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
        std::uint8_t who_am_i = 0;
        if (!speedybee_hw_probe_imu(&who_am_i)) {
            return false;
        }
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
        if (!speedybee_hw_probe_baro(static_cast<std::uint8_t>(speedybee_f405_wing::BARO_ADDR))) {
            return false;
        }
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
        if (!speedybee_hw_probe_pitot()) {
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

    std::uint8_t rx_tmp[64]{};
    const int uart_id = (rx_mode_ == ReceiverMode::CRSF) ? speedybee_f405_wing::CRSF_UART : speedybee_f405_wing::SBUS_UART;
    const int read_count = speedybee_hw_read_uart_bytes(uart_id, rx_tmp, sizeof(rx_tmp));
    if (read_count < 0) {
        return false;
    }

    const std::size_t n = static_cast<std::size_t>(read_count);
    if (n > 0) {
        const std::size_t free_space = rx_stream_buffer_.size() - rx_stream_size_;
        const std::size_t copy_n = (n < free_space) ? n : free_space;
        for (std::size_t i = 0; i < copy_n; ++i) {
            rx_stream_buffer_[rx_stream_size_ + i] = rx_tmp[i];
        }
        rx_stream_size_ += copy_n;
    }

    bool parsed = false;
    while (rx_stream_size_ > 0) {
        std::size_t consumed = 0;
        if (rx_mode_ == ReceiverMode::CRSF) {
            parsed = tryParseCrsf(rx_stream_buffer_.data(), rx_stream_size_, consumed, last_receiver_us_);
        } else {
            parsed = tryParseSbus(rx_stream_buffer_.data(), rx_stream_size_, consumed, last_receiver_us_);
        }

        if (consumed == 0) {
            break;
        }
        consumeBytes(rx_stream_buffer_, rx_stream_size_, consumed);
        if (parsed) {
            out = last_receiver_us_;
            return true;
        }
    }

    return false;
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
