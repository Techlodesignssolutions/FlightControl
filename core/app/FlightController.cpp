#include "FlightController.h"

#include "../types/AircraftState.h"
#include "../types/PilotInput.h"
#include "../types/SensorData.h"

FlightController::FlightController(BoardHAL& hal)
    : hal_(hal) {
}

bool FlightController::init(const LQRScheduleTable& table) {
    if (!hal_.init()) {
        return false;
    }

    if (!controller_.init(table)) {
        return false;
    }

    last_time_us_ = hal_.microsNow();
    return true;
}

void FlightController::update() {
    SensorData sensors{};
    PilotInput pilot{};
    AircraftState state{};

    if (!hal_.readSensors(sensors)) {
        return;
    }
    if (!hal_.readPilotInput(pilot)) {
        return;
    }

    estimator_.update(sensors, state);

    const ControlEffort effort = controller_.update(state, pilot);
    const ActuatorCommand actuators = mixer_.mix(effort);

    hal_.writeActuators(actuators);

    last_time_us_ = hal_.microsNow();
}
