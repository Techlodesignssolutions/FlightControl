#include "SpeedyBeeF405WingBackend.h"

#include "SpeedyBeeF405WingPins.h"

#include <chrono>
#include <cmath>
#include <cstddef>

namespace {
using Clock = std::chrono::steady_clock;
const Clock::time_point kStart = Clock::now();

constexpr int kReceiverModeCrsf = 1;
constexpr int kReceiverModeSbus = 2;
constexpr unsigned long kReceiverTimeoutUs = 120000;

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

// ICM-42688-P assumptions.
constexpr std::uint8_t kImuWhoAmIReg = 0x75;
constexpr std::uint8_t kImuWhoAmIExpected = 0x47;
constexpr std::uint8_t kImuPwrMgmt0 = 0x4E;
constexpr std::uint8_t kImuGyroConfig0 = 0x4F;
constexpr std::uint8_t kImuAccelConfig0 = 0x50;
constexpr std::uint8_t kImuAccelDataX1 = 0x1F;

// SPL06 assumptions.
constexpr std::uint8_t kSpl06ChipIdReg = 0x0D;
constexpr std::uint8_t kSpl06ChipId = 0x10;
constexpr std::uint8_t kSpl06CoefStart = 0x10;
constexpr std::uint8_t kSpl06PrsCfg = 0x06;
constexpr std::uint8_t kSpl06TmpCfg = 0x07;
constexpr std::uint8_t kSpl06MeasCfg = 0x08;
constexpr std::uint8_t kSpl06PrsB2 = 0x00;

constexpr std::size_t kCrsfMinFrameLen = 4;
constexpr std::uint8_t kCrsfTypeRcChannelsPacked = 0x16;
constexpr std::size_t kSbusFrameLen = 25;
constexpr std::uint8_t kSbusStartByte = 0x0F;

float mapRxRawToUs(std::uint16_t raw) {
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
            crc = (crc & 0x80U) ? static_cast<std::uint8_t>((crc << 1U) ^ 0xD5U)
                                : static_cast<std::uint8_t>(crc << 1U);
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

bool tryParseCrsf(const std::uint8_t* buf, std::size_t size, std::size_t& consumed, std::array<int, 8>& out_us) {
    consumed = 0;
    if (size < kCrsfMinFrameLen) {
        return false;
    }

    const std::uint8_t len = buf[1];
    const std::size_t frame_len = static_cast<std::size_t>(len) + 2U;
    if (frame_len < kCrsfMinFrameLen) {
        consumed = 1;
        return false;
    }
    if (size < frame_len) {
        return false;
    }

    const std::uint8_t crc_rx = buf[frame_len - 1U];
    const std::uint8_t crc_calc = crsfCrc8(&buf[2], frame_len - 3U);
    if (crc_rx != crc_calc) {
        consumed = 1;
        return false;
    }

    consumed = frame_len;
    if (buf[2] != kCrsfTypeRcChannelsPacked) {
        return false;
    }

    return unpack11BitChannels(&buf[3], frame_len - 4U, out_us);
}

bool tryParseSbus(const std::uint8_t* buf, std::size_t size, std::size_t& consumed, std::array<int, 8>& out_us) {
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

#if defined(__arm__) || defined(__thumb__)
inline volatile std::uint32_t& reg32(std::uintptr_t addr) {
    return *reinterpret_cast<volatile std::uint32_t*>(addr);
}
#endif

}  // namespace

float SpeedyBeeF405WingBackend::clamp(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

unsigned long SpeedyBeeF405WingBackend::microsNow() {
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - kStart);
    return static_cast<unsigned long>(elapsed.count());
}

void SpeedyBeeF405WingBackend::markError(SubsystemDiag& diag, const char* reason) {
    diag.healthy = false;
    diag.last_error = reason;
    ++diag.error_count;
}

void SpeedyBeeF405WingBackend::markSample(SubsystemDiag& diag, unsigned long now_us) {
    diag.healthy = true;
    diag.last_error = "none";
    diag.last_sample_time_us = now_us;
    ++diag.sample_count;
}

bool SpeedyBeeF405WingBackend::initSpi1Hardware() {
#if defined(__arm__) || defined(__thumb__)
    // Minimal STM32F405 direct-register setup (SPI1 + GPIOA).
    constexpr std::uintptr_t RCC_AHB1ENR = 0x40023830;
    constexpr std::uintptr_t RCC_APB2ENR = 0x40023844;
    constexpr std::uintptr_t GPIOA_MODER = 0x40020000;
    constexpr std::uintptr_t GPIOA_AFRL = 0x40020020;
    constexpr std::uintptr_t GPIOA_BSRR = 0x40020018;
    constexpr std::uintptr_t SPI1_CR1 = 0x40013000;

    reg32(RCC_AHB1ENR) |= (1U << 0);  // GPIOA
    reg32(RCC_APB2ENR) |= (1U << 12); // SPI1
    // PA5/6/7 AF5, PA4 output.
    reg32(GPIOA_MODER) &= ~((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)) | (3U << (4 * 2)));
    reg32(GPIOA_MODER) |= ((2U << (5 * 2)) | (2U << (6 * 2)) | (2U << (7 * 2)) | (1U << (4 * 2)));
    reg32(GPIOA_AFRL) &= ~((0xFU << (5 * 4)) | (0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    reg32(GPIOA_AFRL) |= ((5U << (5 * 4)) | (5U << (6 * 4)) | (5U << (7 * 4)));
    reg32(GPIOA_BSRR) = (1U << 4);

    reg32(SPI1_CR1) = (1U << 2) | (3U << 3) | (1U << 8) | (1U << 9) | (1U << 6); // master + ssm + ssi + br
    return true;
#else
    return false;
#endif
}

bool SpeedyBeeF405WingBackend::initI2c1Hardware() { return false; }
bool SpeedyBeeF405WingBackend::initAdc1Ch15Hardware() { return false; }
bool SpeedyBeeF405WingBackend::initUart1HardwareForCrsf() { return false; }
bool SpeedyBeeF405WingBackend::initUart2HardwareForSbus() { return false; }
bool SpeedyBeeF405WingBackend::initTimerForLogicalPwmChannel(int) { return false; }

bool SpeedyBeeF405WingBackend::imuReadReg(std::uint8_t, std::uint8_t&) { return false; }
bool SpeedyBeeF405WingBackend::imuWriteReg(std::uint8_t, std::uint8_t) { return false; }
bool SpeedyBeeF405WingBackend::imuReadRegs(std::uint8_t, std::uint8_t*, std::size_t) { return false; }

bool SpeedyBeeF405WingBackend::imuProbe() {
    std::uint8_t who = 0;
    if (!imuReadReg(kImuWhoAmIReg, who)) {
        return false;
    }
    imu_whoami_ = who;
    return (who == kImuWhoAmIExpected);
}

bool SpeedyBeeF405WingBackend::imuConfigure() {
    return imuWriteReg(kImuPwrMgmt0, 0x0F) && imuWriteReg(kImuGyroConfig0, 0x06) && imuWriteReg(kImuAccelConfig0, 0x26);
}

bool SpeedyBeeF405WingBackend::imuReadSample(Stm32f4Platform::ImuRaw& out) {
    std::uint8_t raw[12]{};
    if (!imuReadRegs(kImuAccelDataX1, raw, sizeof(raw))) {
        return false;
    }
    auto be16 = [](std::uint8_t hi, std::uint8_t lo) -> std::int16_t {
        return static_cast<std::int16_t>((static_cast<std::uint16_t>(hi) << 8) | lo);
    };
    const std::int16_t ax = be16(raw[0], raw[1]);
    const std::int16_t ay = be16(raw[2], raw[3]);
    const std::int16_t az = be16(raw[4], raw[5]);
    const std::int16_t gx = be16(raw[6], raw[7]);
    const std::int16_t gy = be16(raw[8], raw[9]);
    const std::int16_t gz = be16(raw[10], raw[11]);

    out.gx_rad_s = static_cast<float>(gx) * imu_gyro_scale_rad_s_per_lsb_;
    out.gy_rad_s = static_cast<float>(gy) * imu_gyro_scale_rad_s_per_lsb_;
    out.gz_rad_s = static_cast<float>(gz) * imu_gyro_scale_rad_s_per_lsb_;
    out.ax_m_s2 = static_cast<float>(ax) * imu_accel_scale_m_s2_per_lsb_;
    out.ay_m_s2 = static_cast<float>(ay) * imu_accel_scale_m_s2_per_lsb_;
    out.az_m_s2 = static_cast<float>(az) * imu_accel_scale_m_s2_per_lsb_;
    return true;
}

bool SpeedyBeeF405WingBackend::baroReadReg(std::uint8_t, std::uint8_t&) { return false; }
bool SpeedyBeeF405WingBackend::baroReadRegs(std::uint8_t, std::uint8_t*, std::size_t) { return false; }
bool SpeedyBeeF405WingBackend::baroWriteReg(std::uint8_t, std::uint8_t) { return false; }

bool SpeedyBeeF405WingBackend::baroProbe() {
    std::uint8_t id = 0;
    return baroReadReg(kSpl06ChipIdReg, id) && id == kSpl06ChipId;
}

bool SpeedyBeeF405WingBackend::baroReadCalibration() {
    std::uint8_t coef[18]{};
    if (!baroReadRegs(kSpl06CoefStart, coef, sizeof(coef))) {
        return false;
    }
    c0_ = static_cast<std::int16_t>((coef[0] << 4) | (coef[1] >> 4));
    c1_ = static_cast<std::int16_t>(((coef[1] & 0x0F) << 8) | coef[2]);
    c00_ = static_cast<std::int32_t>((coef[3] << 12) | (coef[4] << 4) | (coef[5] >> 4));
    c10_ = static_cast<std::int32_t>(((coef[5] & 0x0F) << 16) | (coef[6] << 8) | coef[7]);
    c01_ = static_cast<std::int16_t>((coef[8] << 8) | coef[9]);
    c11_ = static_cast<std::int16_t>((coef[10] << 8) | coef[11]);
    c20_ = static_cast<std::int16_t>((coef[12] << 8) | coef[13]);
    c21_ = static_cast<std::int16_t>((coef[14] << 8) | coef[15]);
    c30_ = static_cast<std::int16_t>((coef[16] << 8) | coef[17]);
    baro_cal_loaded_ = true;
    return true;
}

bool SpeedyBeeF405WingBackend::baroConfigure() {
    return baroWriteReg(kSpl06PrsCfg, 0x26) && baroWriteReg(kSpl06TmpCfg, 0xA6) && baroWriteReg(kSpl06MeasCfg, 0x07);
}

bool SpeedyBeeF405WingBackend::baroReadPressureTemp(float& pressure_pa, float& temperature_c) {
    std::uint8_t raw[6]{};
    if (!baroReadRegs(kSpl06PrsB2, raw, sizeof(raw))) {
        return false;
    }
    auto sx24 = [](std::uint32_t v) -> std::int32_t {
        return (v & 0x800000U) ? static_cast<std::int32_t>(v | 0xFF000000U) : static_cast<std::int32_t>(v);
    };
    const float p_sc = static_cast<float>(sx24((raw[0] << 16) | (raw[1] << 8) | raw[2])) / pressure_scale_;
    const float t_sc = static_cast<float>(sx24((raw[3] << 16) | (raw[4] << 8) | raw[5])) / temperature_scale_;

    pressure_pa = static_cast<float>(c00_) + p_sc * (static_cast<float>(c10_) + p_sc * (static_cast<float>(c20_) + p_sc * static_cast<float>(c30_))) +
                  t_sc * static_cast<float>(c01_) + t_sc * p_sc * (static_cast<float>(c11_) + p_sc * static_cast<float>(c21_));
    temperature_c = static_cast<float>(c0_) * 0.5f + static_cast<float>(c1_) * t_sc;
    return true;
}

bool SpeedyBeeF405WingBackend::baroReadAltitude(float& altitude_m) {
    float pressure_pa = 0.0f;
    float temperature_c = 0.0f;
    if (!baroReadPressureTemp(pressure_pa, temperature_c)) {
        return false;
    }
    (void)temperature_c;
    if (pressure_pa <= 1000.0f) {
        return false;
    }
    altitude_m = 44330.0f * (1.0f - std::pow(pressure_pa / sea_level_pressure_pa_, 0.190295f));
    return true;
}

bool SpeedyBeeF405WingBackend::pitotReadRawCounts(std::uint16_t&) { return false; }

bool SpeedyBeeF405WingBackend::pitotZeroAnalog() {
    std::uint32_t sum = 0;
    constexpr int kN = 32;
    for (int i = 0; i < kN; ++i) {
        std::uint16_t c = 0;
        if (!pitotReadRawCounts(c)) {
            return false;
        }
        sum += c;
    }
    pitot_zero_offset_counts_ = static_cast<std::uint16_t>(sum / kN);
    pitot_zeroed_ = true;
    return true;
}

bool SpeedyBeeF405WingBackend::pitotReadAnalogPa(float& dp_pa) {
    std::uint16_t counts = 0;
    if (!pitotReadRawCounts(counts)) {
        return false;
    }
    if (!pitot_zeroed_ && !pitotZeroAnalog()) {
        return false;
    }

    constexpr float vref = 3.3f;
    constexpr float adc_max = 4095.0f;
    const float v = static_cast<float>(counts) * vref / adc_max;
    const float vz = static_cast<float>(pitot_zero_offset_counts_) * vref / adc_max;
    // MPXV7002DP-style approximation around center voltage.
    const float pa = (v - vz) * (2000.0f / 0.9f);

    if (!pitot_filter_initialized_) {
        pitot_filtered_pa_ = pa;
        pitot_filter_initialized_ = true;
    } else {
        pitot_filtered_pa_ += 0.2f * (pa - pitot_filtered_pa_);
    }
    dp_pa = pitot_filtered_pa_;
    return true;
}

int SpeedyBeeF405WingBackend::uart1ReadBytes(std::uint8_t*, std::size_t) { return 0; }
int SpeedyBeeF405WingBackend::uart2ReadBytes(std::uint8_t*, std::size_t) { return 0; }

bool SpeedyBeeF405WingBackend::pwmWriteCompareUs(int, float) { return false; }

std::uint32_t SpeedyBeeF405WingBackend::pwmUsToTicks(float pulse_us) const {
    return static_cast<std::uint32_t>(clamp(pulse_us, 800.0f, 2200.0f) * (static_cast<float>(pwm_timer_tick_hz_) * 1e-6f));
}

bool SpeedyBeeF405WingBackend::ensureImuConfigured() {
    if (imu_diag_.configured) return true;
    if (!imuProbe()) {
        markError(imu_diag_, "imu_probe_failed");
        return false;
    }
    if (!imuConfigure()) {
        markError(imu_diag_, "imu_config_failed");
        return false;
    }
    Stm32f4Platform::ImuRaw first{};
    if (!imuReadSample(first)) {
        markError(imu_diag_, "imu_first_sample_failed");
        return false;
    }
    imu_diag_.configured = true;
    markSample(imu_diag_, microsNow());
    return true;
}

bool SpeedyBeeF405WingBackend::ensureBaroConfigured() {
    if (baro_diag_.configured) return true;
    if (!baroProbe()) {
        markError(baro_diag_, "baro_probe_failed");
        return false;
    }
    if (!baroReadCalibration()) {
        markError(baro_diag_, "baro_coeff_failed");
        return false;
    }
    if (!baroConfigure()) {
        markError(baro_diag_, "baro_config_failed");
        return false;
    }
    float alt = 0.0f;
    if (!baroReadAltitude(alt)) {
        markError(baro_diag_, "baro_first_sample_failed");
        return false;
    }
    baro_diag_.configured = true;
    markSample(baro_diag_, microsNow());
    return true;
}

bool SpeedyBeeF405WingBackend::ensurePitotConfigured() {
    if (pitot_diag_.configured) return true;
    if (!pitotZeroAnalog()) {
        markError(pitot_diag_, "pitot_zero_failed");
        return false;
    }
    float dp = 0.0f;
    if (!pitotReadAnalogPa(dp)) {
        markError(pitot_diag_, "pitot_first_sample_failed");
        return false;
    }
    pitot_diag_.configured = true;
    markSample(pitot_diag_, microsNow());
    return true;
}

bool SpeedyBeeF405WingBackend::ensureReceiverConfigured(int) {
    if (receiver_diag_.configured) return true;
    receiver_diag_.configured = true;
    return true;
}

bool SpeedyBeeF405WingBackend::initSpiBus(int bus_id) {
    if (bus_id != speedybee_f405_wing::IMU_SPI_BUS) {
        markError(imu_diag_, "imu_wrong_spi_bus");
        return false;
    }
    if (spi1_ready_) return true;
    spi1_ready_ = initSpi1Hardware();
    if (!spi1_ready_) markError(imu_diag_, "imu_spi_init_failed");
    return spi1_ready_;
}

bool SpeedyBeeF405WingBackend::initI2cBus(int bus_id) {
    if (bus_id != speedybee_f405_wing::I2C_BUS) {
        markError(baro_diag_, "baro_wrong_i2c_bus");
        return false;
    }
    if (i2c1_ready_) return true;
    i2c1_ready_ = initI2c1Hardware();
    if (!i2c1_ready_) markError(baro_diag_, "baro_i2c_init_failed");
    return i2c1_ready_;
}

bool SpeedyBeeF405WingBackend::initUart(int uart_id, bool inverted_rx) {
    if (uart_id == speedybee_f405_wing::CRSF_UART && !inverted_rx) {
        uart1_ready_ = uart1_ready_ || initUart1HardwareForCrsf();
        if (!uart1_ready_) markError(receiver_diag_, "receiver_uart1_init_failed");
        rx_mode_ = uart1_ready_ ? ReceiverMode::CRSF : ReceiverMode::None;
        return uart1_ready_;
    }
    if (uart_id == speedybee_f405_wing::SBUS_UART && inverted_rx) {
        uart2_ready_ = uart2_ready_ || initUart2HardwareForSbus();
        if (!uart2_ready_) markError(receiver_diag_, "receiver_uart2_init_failed");
        rx_mode_ = uart2_ready_ ? ReceiverMode::SBUS : ReceiverMode::None;
        return uart2_ready_;
    }
    markError(receiver_diag_, "receiver_bad_uart_params");
    return false;
}

bool SpeedyBeeF405WingBackend::initAdcChannel(int adc_id, int channel) {
    if (adc_id != speedybee_f405_wing::AIRSPEED_ADC || channel != speedybee_f405_wing::AIRSPEED_ADC_CHANNEL) {
        markError(pitot_diag_, "pitot_wrong_adc_channel");
        return false;
    }
    if (adc1_ch15_ready_) return true;
    adc1_ch15_ready_ = initAdc1Ch15Hardware();
    if (!adc1_ch15_ready_) markError(pitot_diag_, "pitot_adc_init_failed");
    return adc1_ch15_ready_;
}

bool SpeedyBeeF405WingBackend::initPwmTimerChannel(int timer_id, int channel) {
    for (std::size_t i = 0; i < kPwmTimers.size(); ++i) {
        if (kPwmTimers[i] == timer_id && kPwmChannels[i] == channel) {
            pwm_ready_[i] = pwm_ready_[i] || initTimerForLogicalPwmChannel(static_cast<int>(i));
            if (!pwm_ready_[i]) markError(pwm_diag_, "pwm_channel_init_failed");
            return pwm_ready_[i];
        }
    }
    markError(pwm_diag_, "pwm_bad_timer_channel");
    return false;
}

bool SpeedyBeeF405WingBackend::readImuRaw(Stm32f4Platform::ImuRaw& out) {
    if (!spi1_ready_) {
        markError(imu_diag_, "imu_not_initialized");
        return false;
    }
    if (!ensureImuConfigured()) return false;
    if (!imuReadSample(out)) {
        markError(imu_diag_, "imu_read_failed");
        return false;
    }
    markSample(imu_diag_, microsNow());
    return true;
}

bool SpeedyBeeF405WingBackend::readBaroAltitudeMeters(float& altitude_m) {
    if (!i2c1_ready_) {
        markError(baro_diag_, "baro_not_initialized");
        return false;
    }
    if (!ensureBaroConfigured()) return false;
    if (!baroReadAltitude(altitude_m)) {
        markError(baro_diag_, "baro_read_failed");
        return false;
    }
    markSample(baro_diag_, microsNow());
    return true;
}

bool SpeedyBeeF405WingBackend::readPitotDifferentialPressurePa(float& dp_pa) {
    if (!adc1_ch15_ready_) {
        markError(pitot_diag_, "pitot_not_initialized");
        return false;
    }
    if (!ensurePitotConfigured()) return false;
    if (!pitotReadAnalogPa(dp_pa)) {
        markError(pitot_diag_, "pitot_read_failed");
        return false;
    }
    markSample(pitot_diag_, microsNow());
    return true;
}

bool SpeedyBeeF405WingBackend::readReceiverPulsesUs(std::array<int, 8>& out) {
    if ((rx_mode_ == ReceiverMode::CRSF && !uart1_ready_) ||
        (rx_mode_ == ReceiverMode::SBUS && !uart2_ready_) ||
        rx_mode_ == ReceiverMode::None) {
        markError(receiver_diag_, "receiver_not_initialized");
        return false;
    }

    const int mode = (rx_mode_ == ReceiverMode::CRSF) ? kReceiverModeCrsf : kReceiverModeSbus;
    if (!ensureReceiverConfigured(mode)) return false;

    std::uint8_t rx_tmp[64]{};
    const int read_count = (rx_mode_ == ReceiverMode::CRSF) ? uart1ReadBytes(rx_tmp, sizeof(rx_tmp))
                                                             : uart2ReadBytes(rx_tmp, sizeof(rx_tmp));
    if (read_count < 0) {
        markError(receiver_diag_, "receiver_uart_read_failed");
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
        parsed = (rx_mode_ == ReceiverMode::CRSF)
                     ? tryParseCrsf(rx_stream_buffer_.data(), rx_stream_size_, consumed, last_receiver_us_)
                     : tryParseSbus(rx_stream_buffer_.data(), rx_stream_size_, consumed, last_receiver_us_);
        if (consumed == 0) break;
        consumeBytes(rx_stream_buffer_, rx_stream_size_, consumed);
        if (parsed) {
            out = last_receiver_us_;
            rx_last_frame_time_us_ = microsNow();
            markSample(receiver_diag_, rx_last_frame_time_us_);
            return true;
        }
    }

    const unsigned long now_us = microsNow();
    if (rx_last_frame_time_us_ > 0 && (now_us - rx_last_frame_time_us_) <= kReceiverTimeoutUs) {
        out = last_receiver_us_;
        return true;
    }

    markError(receiver_diag_, "receiver_stale_or_invalid");
    return false;
}

bool SpeedyBeeF405WingBackend::writePwmMicros(int logical_channel, float pulse_us) {
    if (logical_channel < 0 || logical_channel >= static_cast<int>(pwm_ready_.size())) {
        markError(pwm_diag_, "pwm_bad_channel");
        return false;
    }
    if (!pwm_ready_[static_cast<std::size_t>(logical_channel)]) {
        markError(pwm_diag_, "pwm_channel_not_initialized");
        return false;
    }
    if (!pwmWriteCompareUs(logical_channel, pulse_us)) {
        markError(pwm_diag_, "pwm_write_failed");
        return false;
    }

    last_pwm_us_[static_cast<std::size_t>(logical_channel)] = clamp(pulse_us, 800.0f, 2200.0f);
    markSample(pwm_diag_, microsNow());
    return true;
}
