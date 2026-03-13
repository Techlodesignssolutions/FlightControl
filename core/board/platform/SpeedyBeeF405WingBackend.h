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
};
