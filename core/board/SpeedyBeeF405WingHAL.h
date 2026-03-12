#pragma once

#include "BoardHAL.h"

#include <array>
#include <cstdint>

class SpeedyBeeF405WingHAL final : public BoardHAL {
public:
    struct BoardConfig {
        std::uint32_t serial_baud = 115200;
    };

    SpeedyBeeF405WingHAL();
    explicit SpeedyBeeF405WingHAL(const BoardConfig& config);

    bool init() override;
    bool readSensors(SensorData& sensor_data) override;
    bool readPilotInput(PilotInput& pilot_input) override;
    bool writeActuators(const ActuatorCommand& cmd) override;
    unsigned long microsNow() override;

private:
    static float clamp(float x, float lo, float hi);

    BoardConfig config_{};
    bool initialized_{false};

    std::array<float, 4> pilot_channels_{};
    ActuatorCommand last_actuator_cmd_{};
};
