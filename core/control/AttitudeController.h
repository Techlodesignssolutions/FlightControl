#pragma once

#include "ScheduledLQR.h"
#include "../types/AircraftState.h"
#include "../types/AttitudeSetpoint.h"
#include "../types/ControlEffort.h"
#include "../types/PilotInput.h"

class AttitudeController {
public:
    bool init(const LQRScheduleTable& table);

    AttitudeSetpoint makeSetpoint(const PilotInput& pilot) const;
    ControlEffort update(const AircraftState& state, const PilotInput& pilot) const;

private:
    ScheduledLQR lqr_;
};
