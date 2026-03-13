#pragma once

namespace speedybee_f405_wing {

// SPI1 IMU bus.
constexpr int IMU_SPI_BUS = 1;
constexpr int IMU_CS_PORT = 0;   // GPIOA
constexpr int IMU_CS_PIN = 4;    // PA4

// I2C1 barometer / digital pitot bus.
constexpr int I2C_BUS = 1;
constexpr int BARO_ADDR = 0x76;

// Analog airspeed input (PC5 / ADC1 CH15).
constexpr int AIRSPEED_ADC = 1;
constexpr int AIRSPEED_ADC_CHANNEL = 15;

// Receiver UART mappings.
constexpr int CRSF_UART = 1;   // USART1
constexpr int SBUS_UART = 2;   // USART2 RX path (PA3 inverter path)

// PWM channel pin/timer mapping for first four outputs.
constexpr int PWM0_TIMER = 4;  // TIM4_CH2 PB7
constexpr int PWM0_TIM_CHANNEL = 2;
constexpr int PWM1_TIMER = 4;  // TIM4_CH1 PB6
constexpr int PWM1_TIM_CHANNEL = 1;
constexpr int PWM2_TIMER = 3;  // TIM3_CH3 PB0
constexpr int PWM2_TIM_CHANNEL = 3;
constexpr int PWM3_TIMER = 3;  // TIM3_CH4 PB1
constexpr int PWM3_TIM_CHANNEL = 4;

}  // namespace speedybee_f405_wing
