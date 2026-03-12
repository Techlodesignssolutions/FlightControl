#pragma once

#include "LQRScheduleTable.h"
#include "../types/AircraftState.h"
#include "../types/AttitudeSetpoint.h"
#include "../types/ControlEffort.h"

class ScheduledLQR {
public:
    bool init(const LQRScheduleTable& table);

    ControlEffort compute(const AircraftState& state, const AttitudeSetpoint& setpoint) const;

private:
    LQRGainPoint interpolate(float airspeed_mps) const;

    LQRScheduleTable table_{};
};
