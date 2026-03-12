#pragma once

#include "ControlPipelineTypes.h"
#include "HAL.h"
#include "Types.h"
#include "../control/CommandGenerator.h"
#include "../control/ControlMixer.h"
#include "../control/InnerLoopLQR.h"
#include "../control/SafetyManager.h"
#include "../control/ScheduledGainProvider.h"
#include "../control/TECSController.h"
#include "../control/ScheduledLQR.h"
#include "../sensors/AirspeedSensor.h"

class ModularFlightController {
public:
    struct Config {
        const char* lqr_schedule_path = nullptr;
        AirframeType airframe_type = AirframeType::ElevonWing;
    };

    explicit ModularFlightController(HAL* hal);
    ModularFlightController(HAL* hal, const Config& config);

    bool initialize();
    void update();

private:
    FlightState buildFlightState(float dt_s, std::uint32_t now_us);
    PilotInputs readPilotInputs();
    void writeActuators(const ActuatorCommands& act);

    HAL* hal_{nullptr};
    Config config_{};

    ScheduledLQR scheduled_lqr_;
    ScheduledGainProvider gain_provider_;
    CommandGenerator command_generator_;
    InnerLoopLQR lqr_;
    TECSController tecs_;
    SafetyManager safety_;
    ControlMixer mixer_;
    AirspeedSensor airspeed_sensor_;

    std::uint32_t last_loop_us_{0};
};
