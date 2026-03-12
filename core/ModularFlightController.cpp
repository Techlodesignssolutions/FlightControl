#include "ModularFlightController.h"

#include <algorithm>

namespace {

float clampf(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}

}  // namespace

ModularFlightController::ModularFlightController(HAL* hal)
    : ModularFlightController(hal, Config{}) {
}

ModularFlightController::ModularFlightController(HAL* hal, const Config& config)
    : hal_(hal)
    , config_(config)
    , scheduled_lqr_()
    , gain_provider_()
    , command_generator_(CommandGenerator::Config{})
    , lqr_(&gain_provider_, InnerLoopLQR::Config{})
    , tecs_()
    , safety_(SafetyManager::Config{})
    , mixer_(config.airframe_type)
    , airspeed_sensor_()
    , last_loop_us_(0) {
}

bool ModularFlightController::initialize() {
    if (hal_ == nullptr) {
        return false;
    }

    ScheduledLQR::ScheduleConfig schedule;
    if (!scheduled_lqr_.loadFromJsonFile(config_.lqr_schedule_path, schedule, nullptr)) {
        return false;
    }
    if (!scheduled_lqr_.configure(schedule, nullptr)) {
        return false;
    }
    if (!gain_provider_.configure(schedule, schedule.controller)) {
        return false;
    }

    if (!hal_->initIMU() || !hal_->initRadio() || !hal_->initServos() || !hal_->initMotors()) {
        return false;
    }
    airspeed_sensor_.init();
    tecs_.init();
    lqr_.init();
    last_loop_us_ = hal_->micros();
    return true;
}

FlightState ModularFlightController::buildFlightState(float dt_s, std::uint32_t now_us) {
    FlightState s;
    s.dt_s = dt_s;
    s.t_s = now_us * 1e-6f;

    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 9.80665f};
    float mag[3] = {1.0f, 0.0f, 0.0f};
    if (hal_->readIMU(gyro, accel, mag)) {
        s.p_rad_s = gyro[0];
        s.q_rad_s = gyro[1];
        s.r_rad_s = gyro[2];
        s.roll_rad += s.p_rad_s * dt_s;
        s.pitch_rad += s.q_rad_s * dt_s;
        s.yaw_rad += s.r_rad_s * dt_s;
    }

    const AirspeedEstimate air = airspeed_sensor_.update(dt_s, 0.0f);
    s.airspeed_mps = air.airspeed_mps;
    s.airspeed_valid = air.valid;

    s.imu_valid = hal_->isIMUHealthy();
    s.attitude_valid = true;
    s.radio_valid = hal_->isRadioConnected();
    s.actuator_valid = true;
    return s;
}

PilotInputs ModularFlightController::readPilotInputs() {
    PilotInputs p;
    float ch[6] = {0};
    if (hal_->readRadio(ch, 6)) {
        p.throttle_cmd_0to1 = clampf(ch[0], 0.0f, 1.0f);
        p.roll_cmd_norm = clampf(ch[1], -1.0f, 1.0f);
        p.pitch_cmd_norm = clampf(ch[2], -1.0f, 1.0f);
        p.yaw_cmd_norm = clampf(ch[3], -1.0f, 1.0f);
    }
    return p;
}

void ModularFlightController::update() {
    const std::uint32_t now_us = hal_->micros();
    const float dt_s = (now_us - last_loop_us_) * 1e-6f;
    last_loop_us_ = now_us;

    const FlightState state = buildFlightState(dt_s, now_us);
    const PilotInputs pilot = readPilotInputs();

    InnerLoopCommands cmd;
    if (pilot.tecs_mode) {
        OuterLoopCommands outer;
        outer.roll_cmd_rad = pilot.roll_cmd_norm * gain_provider_.maxRollCommandRad();
        outer.target_altitude_m = state.altitude_m;
        outer.target_airspeed_mps = safety_.safeScheduledAirspeed(state);

        const TECSOutputs tecs_out = tecs_.update(state, outer);
        cmd = command_generator_.updateFromOuterLoop(outer.roll_cmd_rad,
                                                     tecs_out.pitch_cmd_rad,
                                                     tecs_out.throttle_cmd_0to1,
                                                     state);
    } else {
        cmd = command_generator_.updateFromPilot(pilot, state);
    }

    if (!safety_.canRunClosedLoop(state)) {
        InnerLoopOutputs safe_out;
        safe_out.throttle_cmd_0to1 = cmd.throttle_cmd_0to1;
        writeActuators(mixer_.mix(safe_out));
        return;
    }

    InnerLoopOutputs u = lqr_.update(state, cmd);
    safety_.applyOutputLimits(u, dt_s);
    writeActuators(mixer_.mix(u));
}

void ModularFlightController::writeActuators(const ActuatorCommands& act) {
    hal_->writeServo(0, act.left_surface_0to1);
    hal_->writeServo(1, act.right_surface_0to1);
    hal_->writeServo(2, act.rudder_0to1);
    hal_->writeMotor(0, act.throttle_0to1);
}
