// ---------- FlightController.cpp ----------
#include "FlightController.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace {

constexpr float kPi = 3.14159265358979323846f;

float clampf(float value, float lo, float hi) {
    return std::max(lo, std::min(hi, value));
}

}  // namespace

// Constructor
FlightController::FlightController(HAL* hal, const Config& config)
    : hal_(hal)
    , config_(config)
    , status_(Status::INITIALIZING)
    , control_mode_(config.default_mode)
    , state_estimator_(hal_, StateEstimator::Config{})
    , scheduled_lqr_()
    , adaptive_augmentor_(AdaptiveAugmentor::Config{})
    , safety_governor_(SafetyGovernor::Config{})
    , mixer_(hal_, FixedWingMixer::Config{})
    , loop_start_time_(0)
    , last_loop_time_(0)
    , loop_dt_(0.0f)
    , last_mode_change_ms_(0) {
    performance_stats_ = PerformanceStats{ 0.0f, 0.0f, 0, 0, 0.0f };
    for (int i = 0; i < TIMING_SAMPLES; ++i) {
        loop_time_history_[i] = 0.0f;
    }
}

bool FlightController::initializeControlStack() {
    ScheduledLQR::ScheduleConfig schedule;
    std::string error;

    if (!scheduled_lqr_.loadFromJsonFile(config_.lqr_schedule_path, schedule, &error)) {
        if (hal_) {
            hal_->serialPrint("LQR schedule load failed: ");
            hal_->serialPrintln(error.c_str());
        }
        return false;
    }

    if (!scheduled_lqr_.configure(schedule, &error)) {
        if (hal_) {
            hal_->serialPrint("LQR schedule configure failed: ");
            hal_->serialPrintln(error.c_str());
        }
        return false;
    }

    AdaptiveAugmentor::Config augmentor_cfg;
    augmentor_cfg.learning_rate = schedule.adaptive.learning_rate;
    augmentor_cfg.leakage = schedule.adaptive.leakage;
    augmentor_cfg.weight_limit = schedule.adaptive.weight_limit;
    augmentor_cfg.output_limit_roll = schedule.adaptive.output_limit_roll;
    augmentor_cfg.output_limit_pitch = schedule.adaptive.output_limit_pitch;
    augmentor_cfg.output_limit_yaw = schedule.adaptive.output_limit_yaw;
    adaptive_augmentor_.configure(augmentor_cfg);
    adaptive_augmentor_.reset();
    adaptive_augmentor_.freeze(false);

    SafetyGovernor::Config safety_cfg;
    safety_cfg.adaptive_limit_roll = schedule.safety.adaptive_limit_roll;
    safety_cfg.adaptive_limit_pitch = schedule.safety.adaptive_limit_pitch;
    safety_cfg.adaptive_limit_yaw = schedule.safety.adaptive_limit_yaw;
    safety_cfg.adaptive_rate_limit_roll = schedule.safety.adaptive_rate_limit_roll;
    safety_cfg.adaptive_rate_limit_pitch = schedule.safety.adaptive_rate_limit_pitch;
    safety_cfg.adaptive_rate_limit_yaw = schedule.safety.adaptive_rate_limit_yaw;
    safety_cfg.total_limit_roll = schedule.safety.total_limit_roll;
    safety_cfg.total_limit_pitch = schedule.safety.total_limit_pitch;
    safety_cfg.total_limit_yaw = schedule.safety.total_limit_yaw;
    safety_governor_.configure(safety_cfg);
    safety_governor_.reset();

    control_stack_ready_ = true;
    return true;
}

bool FlightController::initialize() {
    if (!hal_) {
        return false;
    }

    if (!hal_->initIMU() || !hal_->initRadio() || !hal_->initServos() || !hal_->initMotors()) {
        status_ = Status::SYSTEM_ERROR;
        return false;
    }

    if (!state_estimator_.initialize() || !mixer_.initialize() || !initializeControlStack()) {
        status_ = Status::SYSTEM_ERROR;
        return false;
    }

    status_ = Status::READY;
    last_loop_time_ = hal_->micros();
    last_mode_change_ms_ = hal_->millis();
    previous_radio_inputs_ = radio_inputs_;
    previous_airspeed_ = 0.0f;

    return true;
}

void FlightController::update() {
    loop_start_time_ = hal_->micros();
    std::uint32_t dt_us = loop_start_time_ - last_loop_time_;
    loop_dt_ = dt_us / 1e6f;

    if (loop_dt_ < 0.001f) {
        return;
    }

    last_loop_time_ = loop_start_time_;

    if (!readRadioInputs() || !updateStateEstimation() || !runControlLoops() || !mixAndOutputControls()) {
        return;
    }

    updateSafetyMonitoring();
    updatePerformanceStats();
    outputDebugInfo();
    ++performance_stats_.loop_count;
}

AircraftState FlightController::getAircraftState() const {
    return aircraft_state_;
}

bool FlightController::setControlMode(ControlMode mode) {
    if (status_ != Status::READY && status_ != Status::ARMED && status_ != Status::FLYING) {
        return false;
    }

    control_mode_ = mode;
    last_mode_change_ms_ = hal_ ? hal_->millis() : 0;
    safety_governor_.reset();
    return true;
}

void FlightController::emergencyStop() {
    mixer_.emergencyStop();
    status_ = Status::EMERGENCY;
    armed_ = false;
    emergency_mode_ = true;
    adaptive_augmentor_.freeze(true);
}

bool FlightController::arm() {
    if (status_ != Status::READY) {
        return false;
    }

    if (!checkSystemHealth()) {
        return false;
    }

    if (battery_voltage_ < config_.low_battery_voltage + 0.5f) {
        return false;
    }

    if (!isRadioSignalValid()) {
        return false;
    }

    if (radio_inputs_.throttle > 0.1f) {
        return false;
    }

    armed_ = true;
    status_ = Status::ARMED;
    emergency_mode_ = false;
    adaptive_augmentor_.freeze(false);
    mixer_.clearEmergency();

    if (hal_) {
        hal_->serialPrintln("ARMED - Flight controller ready");
    }

    return true;
}

bool FlightController::disarm() {
    if (status_ == Status::FLYING) {
        return false;
    }

    armed_ = false;
    status_ = Status::READY;
    mixer_.emergencyStop();

    if (hal_) {
        hal_->serialPrintln("DISARMED - Motors stopped");
    }

    return true;
}

DiagnosticInfo FlightController::getDiagnosticInfo() const {
    DiagnosticInfo info;
    info.imu_healthy = true;
    info.radio_healthy = hal_->isRadioConnected();
    info.airspeed_converged = state_estimator_.getAirspeedState().converged;
    info.adaptive_learning = adaptive_augmentor_.isLearning();
    info.uptime_ms = hal_->millis();
    return info;
}

bool FlightController::readRadioInputs() {
    float channels[6];
    if (!hal_->readRadio(channels, 6)) {
        return false;
    }

    radio_inputs_.throttle = clampf(channels[0], 0.0f, 1.0f);
    radio_inputs_.roll = clampf(channels[1], -1.0f, 1.0f);
    radio_inputs_.pitch = clampf(channels[2], -1.0f, 1.0f);
    radio_inputs_.yaw = clampf(channels[3], -1.0f, 1.0f);
    radio_inputs_.aux1 = clampf(channels[4], -1.0f, 1.0f);
    radio_inputs_.aux2 = clampf(channels[5], -1.0f, 1.0f);
    radio_inputs_.last_update_time = hal_->millis();
    last_radio_update_ = hal_->millis();
    return true;
}

bool FlightController::updateStateEstimation() {
    state_estimator_.setThrottleCommand(radio_inputs_.throttle);
    state_estimator_.update(loop_dt_);
    aircraft_state_ = state_estimator_.getState();
    return true;
}

bool FlightController::runControlLoops() {
    switch (control_mode_) {
    case ControlMode::MANUAL:
        runManualMode();
        break;
    case ControlMode::STABILIZE:
        runStabilizeMode();
        break;
    case ControlMode::ALTITUDE:
        runAltitudeMode();
        break;
    case ControlMode::POSITION:
        runPositionMode();
        break;
    case ControlMode::AUTO:
        runAutoMode();
        break;
    }
    return true;
}

bool FlightController::mixAndOutputControls() {
    RadioInputs safe_radio_inputs = radio_inputs_;
    if (!armed_ || status_ == Status::EMERGENCY) {
        safe_radio_inputs.throttle = 0.0f;
    }

    control_outputs_ = mixer_.mix(control_commands_, safe_radio_inputs);
    mixer_.applyControls(control_outputs_);

    if (armed_ && safe_radio_inputs.throttle > 0.1f) {
        if (status_ == Status::ARMED) {
            status_ = Status::FLYING;
        }
    } else if (status_ == Status::FLYING && safe_radio_inputs.throttle < 0.05f) {
        status_ = Status::ARMED;
    }

    return true;
}

void FlightController::updateSafetyMonitoring() {
    battery_voltage_ = readBatteryVoltage();
    if (!isRadioSignalValid()) {
        handleRadioTimeout();
    }
    if (battery_voltage_ < config_.low_battery_voltage) {
        handleBatteryLow();
    }
}

void FlightController::updatePerformanceStats() {
    float ms = (hal_->micros() - loop_start_time_) / 1e3f;
    updateLoopTimingStats(ms);

    if (ms > config_.max_loop_time_ms) {
        ++performance_stats_.overrun_count;
    }

    performance_stats_.loop_time_max_ms =
        (performance_stats_.loop_time_max_ms > ms ? performance_stats_.loop_time_max_ms : ms);

    performance_stats_.cpu_usage_percent = (ms / (1000.0f / config_.loop_frequency_hz)) * 100.0f;
}

void FlightController::outputDebugInfo() {
    if (!config_.enable_debug_output) {
        return;
    }

    std::uint32_t now = hal_->millis();
    if (now - last_debug_output_ < (1000 / config_.debug_output_rate_hz)) {
        return;
    }

    char buf[128];
    std::snprintf(buf,
                  sizeof(buf),
                  "Status:%d Loop:%.2fms Learn:%d Sat:%d r_cmd:%.3f",
                  static_cast<int>(status_),
                  performance_stats_.loop_time_avg_ms,
                  last_learning_enabled_ ? 1 : 0,
                  last_saturation_ ? 1 : 0,
                  last_r_cmd_);
    hal_->serialPrintln(buf);
    last_debug_output_ = now;
}

bool FlightController::checkSystemHealth() {
    if (!hal_->isIMUHealthy()) {
        if (hal_) {
            hal_->serialPrintln("HEALTH: IMU failure");
        }
        return false;
    }

    if (!hal_->isRadioConnected()) {
        if (hal_) {
            hal_->serialPrintln("HEALTH: Radio disconnected");
        }
        return false;
    }

    if (!state_estimator_.getAirspeedState().converged) {
        if (hal_) {
            hal_->serialPrintln("HEALTH: Airspeed estimator not converged");
        }
        return false;
    }

    if (!control_stack_ready_) {
        if (hal_) {
            hal_->serialPrintln("HEALTH: Control stack not initialized");
        }
        return false;
    }

    if (isInEmergencyCondition()) {
        if (hal_) {
            hal_->serialPrintln("HEALTH: Emergency condition active");
        }
        return false;
    }

    return true;
}

void FlightController::handleRadioTimeout() {
    if (!radio_timeout_active_) {
        radio_timeout_start_ = hal_->millis();
        radio_timeout_active_ = true;
    }

    if (hal_->millis() - radio_timeout_start_ > config_.radio_timeout_ms) {
        emergencyStop();
    }
}

void FlightController::handleBatteryLow() {
    status_ = Status::EMERGENCY;
    emergency_mode_ = true;

    ControlSurfaces emergency_controls;
    emergency_controls.throttle = 0.0f;
    emergency_controls.left_elevon = 0.0f;
    emergency_controls.right_elevon = 0.0f;
    emergency_controls.rudder = 0.0f;

    mixer_.emergencyStop();
    mixer_.applyControls(emergency_controls);

    if (hal_) {
        char msg[96];
        std::snprintf(msg, sizeof(msg), "BATTERY LOW: %.2fV - EMERGENCY LANDING", battery_voltage_);
        hal_->serialPrintln(msg);
    }

    adaptive_augmentor_.freeze(true);
}


bool FlightController::isInEmergencyCondition() const {
    return status_ == Status::EMERGENCY || status_ == Status::SYSTEM_ERROR;
}

void FlightController::runManualMode() {
    control_commands_.roll_rate = radio_inputs_.roll * kPi;            // +/- 180 deg/s
    control_commands_.pitch_rate = radio_inputs_.pitch * (0.5f * kPi); // +/- 90 deg/s
    control_commands_.yaw_rate = radio_inputs_.yaw * (0.5f * kPi);     // +/- 90 deg/s

    adaptive_augmentor_.update(loop_dt_, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
    previous_radio_inputs_ = radio_inputs_;
    previous_airspeed_ = aircraft_state_.airspeed;
    last_learning_enabled_ = false;
    last_saturation_ = false;
}

float FlightController::calculateStickStepMagnitude() const {
    float step = 0.0f;
    step = std::max(step, std::fabs(radio_inputs_.roll - previous_radio_inputs_.roll));
    step = std::max(step, std::fabs(radio_inputs_.pitch - previous_radio_inputs_.pitch));
    step = std::max(step, std::fabs(radio_inputs_.yaw - previous_radio_inputs_.yaw));
    step = std::max(step, std::fabs(radio_inputs_.throttle - previous_radio_inputs_.throttle));
    return step;
}

bool FlightController::shouldEnableLearning(const SafetyGovernor::Result& governor_result, float stick_step_mag) const {
    const auto& adaptive_cfg = scheduled_lqr_.adaptiveParams();

    const bool estimator_ok = hal_->isIMUHealthy();
    const bool airspeed_valid = std::isfinite(aircraft_state_.airspeed) != 0 && aircraft_state_.airspeed > 1.0f;
    const bool airspeed_converged = state_estimator_.getAirspeedState().converged;
    const bool near_stall = aircraft_state_.airspeed < adaptive_cfg.near_stall_airspeed;
    const bool surface_saturated = governor_result.adaptive_saturated || governor_result.total_saturated;
    const bool recent_mode_change =
        (hal_->millis() - last_mode_change_ms_) < static_cast<std::uint32_t>(adaptive_cfg.mode_change_freeze_s * 1000.0f);
    const bool large_stick_step = stick_step_mag > adaptive_cfg.stick_step_threshold;
    const bool regime_stable = std::fabs(aircraft_state_.airspeed - previous_airspeed_) < adaptive_cfg.regime_stability_delta;
    const bool emergency_active = isInEmergencyCondition();

    return estimator_ok && airspeed_valid && airspeed_converged && !near_stall && !surface_saturated &&
           !recent_mode_change && !large_stick_step && regime_stable && !emergency_active;
}

void FlightController::runClosedLoopAttitudeMode() {
    if (!control_stack_ready_) {
        control_commands_ = AngularRates{};
        last_learning_enabled_ = false;
        last_saturation_ = false;
        return;
    }


    const float phi = aircraft_state_.attitude.roll;
    const float theta = aircraft_state_.attitude.pitch;
    const float p = aircraft_state_.rates.roll_rate;
    const float q = aircraft_state_.rates.pitch_rate;
    const float r = aircraft_state_.rates.yaw_rate;
    const float airspeed = std::max(0.0f, aircraft_state_.airspeed);

    const float phi_cmd = radio_inputs_.roll * scheduled_lqr_.maxRollCommandRad();
    const float theta_cmd = radio_inputs_.pitch * scheduled_lqr_.maxPitchCommandRad();

    const float k_coord = scheduled_lqr_.interpolateYawCoordinationGain(airspeed);
    const float r_cmd = scheduled_lqr_.yawStickGain() * radio_inputs_.yaw + k_coord * phi_cmd;

    const float e_phi = phi_cmd - phi;
    const float e_theta = theta_cmd - theta;
    const float e_r = r_cmd - r;

    const float ua_base = scheduled_lqr_.computeRoll(airspeed, phi, p, phi_cmd);
    const float ue_base = scheduled_lqr_.computePitch(airspeed, theta, q, theta_cmd);
    const float ur_base = scheduled_lqr_.computeYaw(airspeed, r, r_cmd);

    const AdaptiveAugmentor::Output adapt = adaptive_augmentor_.compute(e_phi, p, e_theta, q, e_r, r);

    SafetyGovernor::AxisValues base;
    base.roll = ua_base;
    base.pitch = ue_base;
    base.yaw = ur_base;

    SafetyGovernor::AxisValues adapt_axis;
    adapt_axis.roll = adapt.ua;
    adapt_axis.pitch = adapt.ue;
    adapt_axis.yaw = adapt.ur;

    const SafetyGovernor::Result governed = safety_governor_.apply(loop_dt_, base, adapt_axis);

    control_commands_.roll_rate = governed.total.roll;
    control_commands_.pitch_rate = governed.total.pitch;
    control_commands_.yaw_rate = governed.total.yaw;

    const float stick_step = calculateStickStepMagnitude();
    const bool learning_enabled = shouldEnableLearning(governed, stick_step);

    adaptive_augmentor_.update(loop_dt_, e_phi, p, e_theta, q, e_r, r, learning_enabled);

    previous_radio_inputs_ = radio_inputs_;
    previous_airspeed_ = airspeed;
    last_learning_enabled_ = learning_enabled;
    last_saturation_ = governed.adaptive_saturated || governed.total_saturated;
    last_r_cmd_ = r_cmd;
}

void FlightController::runStabilizeMode() {
    runClosedLoopAttitudeMode();
}

void FlightController::runAltitudeMode() {
    runClosedLoopAttitudeMode();
}

void FlightController::runPositionMode() {
    runClosedLoopAttitudeMode();
}

void FlightController::runAutoMode() {
    runClosedLoopAttitudeMode();
}


void FlightController::updateLoopTimingStats(float ms) {
    if (performance_stats_.loop_count == 0) {
        performance_stats_.loop_time_avg_ms = ms;
        performance_stats_.loop_time_max_ms = ms;
    } else {
        const float alpha = 0.1f;
        performance_stats_.loop_time_avg_ms = alpha * ms + (1 - alpha) * performance_stats_.loop_time_avg_ms;
        performance_stats_.loop_time_max_ms =
            (performance_stats_.loop_time_max_ms > ms ? performance_stats_.loop_time_max_ms : ms);
    }

    ++performance_stats_.loop_count;
    loop_time_history_[timing_index_] = ms;
    timing_index_ = (timing_index_ + 1) % TIMING_SAMPLES;
}


bool FlightController::isRadioSignalValid() const {
    return (hal_->millis() - last_radio_update_) < config_.radio_timeout_ms;
}

float FlightController::readBatteryVoltage() const {
    return 11.1f;
}

bool FlightController::loadConfiguration(const void* data, std::size_t size) {
    struct PackedConfig {
        std::uint32_t loop_frequency_hz;
        std::uint32_t default_mode;
        float max_attitude_error;
        float radio_timeout_ms;
        float low_battery_voltage;
        float max_loop_time_ms;
        std::uint32_t enable_loop_timing_monitoring;
        std::uint32_t enable_debug_output;
        std::uint32_t debug_output_rate_hz;
    };

    if (data == nullptr || size < sizeof(PackedConfig)) {
        return false;
    }

    PackedConfig packed;
    std::memcpy(&packed, data, sizeof(PackedConfig));

    config_.loop_frequency_hz = packed.loop_frequency_hz;
    config_.default_mode = static_cast<ControlMode>(packed.default_mode);
    config_.max_attitude_error = packed.max_attitude_error;
    config_.radio_timeout_ms = packed.radio_timeout_ms;
    config_.low_battery_voltage = packed.low_battery_voltage;
    config_.max_loop_time_ms = packed.max_loop_time_ms;
    config_.enable_loop_timing_monitoring = packed.enable_loop_timing_monitoring != 0;
    config_.enable_debug_output = packed.enable_debug_output != 0;
    config_.debug_output_rate_hz = packed.debug_output_rate_hz;

    return true;
}

bool FlightController::saveConfiguration(void* data, std::size_t* size) const {
    struct PackedConfig {
        std::uint32_t loop_frequency_hz;
        std::uint32_t default_mode;
        float max_attitude_error;
        float radio_timeout_ms;
        float low_battery_voltage;
        float max_loop_time_ms;
        std::uint32_t enable_loop_timing_monitoring;
        std::uint32_t enable_debug_output;
        std::uint32_t debug_output_rate_hz;
    };

    if (size == nullptr) {
        return false;
    }

    *size = sizeof(PackedConfig);
    if (data == nullptr) {
        return true;
    }

    PackedConfig packed{};
    packed.loop_frequency_hz = config_.loop_frequency_hz;
    packed.default_mode = static_cast<std::uint32_t>(config_.default_mode);
    packed.max_attitude_error = config_.max_attitude_error;
    packed.radio_timeout_ms = config_.radio_timeout_ms;
    packed.low_battery_voltage = config_.low_battery_voltage;
    packed.max_loop_time_ms = config_.max_loop_time_ms;
    packed.enable_loop_timing_monitoring = config_.enable_loop_timing_monitoring ? 1u : 0u;
    packed.enable_debug_output = config_.enable_debug_output ? 1u : 0u;
    packed.debug_output_rate_hz = config_.debug_output_rate_hz;

    std::memcpy(data, &packed, sizeof(PackedConfig));
    return true;
}





