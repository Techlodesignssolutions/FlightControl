#include "AttitudeController.h"

bool AttitudeController::init(const LQRScheduleTable& table) {
    return lqr_.init(table);
}

AttitudeSetpoint AttitudeController::makeSetpoint(const PilotInput& pilot) const {
    constexpr float kMaxRollRad = 0.52f;
    constexpr float kMaxPitchRad = 0.35f;

    AttitudeSetpoint sp{};
    sp.roll_rad = pilot.roll_cmd * kMaxRollRad;
    sp.pitch_rad = pilot.pitch_cmd * kMaxPitchRad;
    sp.yaw_cmd = pilot.yaw_cmd;
    sp.throttle_cmd = pilot.throttle_cmd;
    return sp;
}

ControlEffort AttitudeController::update(const AircraftState& state, const PilotInput& pilot) const {
    const AttitudeSetpoint setpoint = makeSetpoint(pilot);
    return lqr_.compute(state, setpoint);
}
