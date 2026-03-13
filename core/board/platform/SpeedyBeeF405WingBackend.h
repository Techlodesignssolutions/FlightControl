#pragma once

#include "Stm32f4Platform.h"

#include <array>
#include <cstddef>
#include <cstdint>

class SpeedyBeeF405WingBackend final : public Stm32f4Platform::Backend {
public:
    struct SubsystemDiag {
        bool configured{false};
        bool healthy{false};
        unsigned long last_sample_time_us{0};
        std::uint32_t sample_count{0};
        std::uint32_t error_count{0};
        const char* last_error{"none"};
    };

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

    const SubsystemDiag& imuDiag() const { return imu_diag_; }
    const SubsystemDiag& baroDiag() const { return baro_diag_; }
    const SubsystemDiag& pitotDiag() const { return pitot_diag_; }
    const SubsystemDiag& receiverDiag() const { return receiver_diag_; }
    const SubsystemDiag& pwmDiag() const { return pwm_diag_; }

private:
    enum class ReceiverMode {
        None,
        CRSF,
        SBUS
    };

    static float clamp(float x, float lo, float hi);
    static unsigned long microsNow();

    // Real backend helpers.
    bool initSpi1Hardware();
    bool initI2c1Hardware();
    bool initAdc1Ch15Hardware();
    bool initUart1HardwareForCrsf();
    bool initUart2HardwareForSbus();
    bool initTimerForLogicalPwmChannel(int logical_channel);

    bool imuReadReg(std::uint8_t reg, std::uint8_t& value);
    bool imuWriteReg(std::uint8_t reg, std::uint8_t value);
    bool imuReadRegs(std::uint8_t start_reg, std::uint8_t* dst, std::size_t len);
    bool imuProbe();
    bool imuConfigure();
    bool imuReadSample(Stm32f4Platform::ImuRaw& out);

    bool baroReadReg(std::uint8_t reg, std::uint8_t& value);
    bool baroReadRegs(std::uint8_t reg, std::uint8_t* dst, std::size_t len);
    bool baroWriteReg(std::uint8_t reg, std::uint8_t value);
    bool baroProbe();
    bool baroReadCalibration();
    bool baroConfigure();
    bool baroReadPressureTemp(float& pressure_pa, float& temperature_c);
    bool baroReadAltitude(float& altitude_m);

    bool pitotReadRawCounts(std::uint16_t& counts);
    bool pitotZeroAnalog();
    bool pitotReadAnalogPa(float& dp_pa);

    int uart1ReadBytes(std::uint8_t* out, std::size_t max_len);
    int uart2ReadBytes(std::uint8_t* out, std::size_t max_len);

    bool pwmWriteCompareUs(int logical_channel, float pulse_us);
    std::uint32_t pwmUsToTicks(float pulse_us) const;

    bool ensureImuConfigured();
    bool ensureBaroConfigured();
    bool ensurePitotConfigured();
    bool ensureReceiverConfigured(int mode);

    static void markError(SubsystemDiag& diag, const char* reason);
    static void markSample(SubsystemDiag& diag, unsigned long now_us);

    bool spi1_ready_{false};
    bool i2c1_ready_{false};
    bool uart1_ready_{false};
    bool uart2_ready_{false};
    bool adc1_ch15_ready_{false};

    std::array<bool, 4> pwm_ready_{{false, false, false, false}};
    std::array<float, 8> last_pwm_us_{{1500.0f, 1500.0f, 1500.0f, 1000.0f, 1500.0f, 1500.0f, 1500.0f, 1500.0f}};

    // IMU assumptions: ICM-42688-P register map on SPI1.
    std::uint8_t imu_whoami_{0};
    float imu_gyro_scale_rad_s_per_lsb_{0.0010642f};
    float imu_accel_scale_m_s2_per_lsb_{0.0023942f};

    // SPL06 calibration and conversion state on I2C1 address 0x76.
    bool baro_cal_loaded_{false};
    std::int16_t c0_{0};
    std::int16_t c1_{0};
    std::int32_t c00_{0};
    std::int32_t c10_{0};
    std::int16_t c01_{0};
    std::int16_t c11_{0};
    std::int16_t c20_{0};
    std::int16_t c21_{0};
    std::int16_t c30_{0};
    float pressure_scale_{253952.0f};
    float temperature_scale_{253952.0f};
    float sea_level_pressure_pa_{101325.0f};

    // Analog pitot assumptions: MPXV7002DP-style centered transfer.
    bool pitot_zeroed_{false};
    std::uint16_t pitot_zero_offset_counts_{0};
    float pitot_filtered_pa_{0.0f};
    bool pitot_filter_initialized_{false};

    ReceiverMode rx_mode_{ReceiverMode::None};
    std::array<int, 8> last_receiver_us_{{1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500}};

    std::array<std::uint8_t, 128> rx_stream_buffer_{};
    std::size_t rx_stream_size_{0};

    SubsystemDiag imu_diag_{};
    SubsystemDiag baro_diag_{};
    SubsystemDiag pitot_diag_{};
    SubsystemDiag receiver_diag_{};
    SubsystemDiag pwm_diag_{};

    unsigned long rx_last_frame_time_us_{0};
    std::uint32_t pwm_timer_tick_hz_{1000000U};
};
