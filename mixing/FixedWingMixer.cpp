#include "FixedWingMixer.h"

#include <algorithm>

#ifdef _WIN32
#define _USE_MATH_DEFINES
#include <cmath>
#endif

FixedWingMixer::FixedWingMixer(HAL* hal, const Config& config)
    : hal_(hal)
    , config_(config)
    , initialized_(false)
    , throttle_passthrough_(false)
    , emergency_mode_(false)
    , current_controls_{ 0.0f, 0.0f, 0.0f, 0.0f } {
    left_elevon_channel_ = config_.left_elevon_channel;
    right_elevon_channel_ = config_.right_elevon_channel;
    rudder_channel_ = config_.rudder_channel;
    throttle_channel_ = config_.motor_channel;

    left_elevon_trim_ = 0.0f;
    right_elevon_trim_ = 0.0f;
    rudder_trim_ = 0.0f;

    history_index_ = 0;
    for (int i = 0; i < RATE_LIMIT_SAMPLES; ++i) {
        left_elevon_history_[i] = 0.0f;
        right_elevon_history_[i] = 0.0f;
        rudder_history_[i] = 0.0f;
    }

    last_update_time_ = 0;
}

bool FixedWingMixer::initialize() {
    if (!hal_) {
        return false;
    }

    if (initialized_) {
        return true;
    }

    initialized_ = hal_->initServos() && hal_->initMotors();
    if (initialized_) {
        emergencyStop();
        clearEmergency();
        last_update_time_ = hal_->millis();
    }

    return initialized_;
}

ControlSurfaces FixedWingMixer::mix(const AngularRates& controller_outputs,
                                    const RadioInputs& radio_inputs) {
    if (!initialized_) {
        return ControlSurfaces();
    }

    const uint32_t current_time = hal_->millis();
    const float dt = (current_time - last_update_time_) / 1000.0f;
    last_update_time_ = current_time;

    ControlSurfaces controls = mixControls(controller_outputs, radio_inputs);
    applySafetyLimits(controls);
    applyRateLimiting(controls, dt);

    current_controls_ = controls;
    return controls;
}

void FixedWingMixer::applyControls(const ControlSurfaces& controls) {
    if (!hal_ || !initialized_ || emergency_mode_) {
        return;
    }

    current_controls_ = controls;
    writeActuators(controls);
}

void FixedWingMixer::emergencyStop() {
    if (!hal_) {
        return;
    }

    emergency_mode_ = true;
    ControlSurfaces safe_controls{ 0.0f, 0.0f, 0.0f, 0.0f };
    writeActuators(safe_controls);
    current_controls_ = safe_controls;
}

void FixedWingMixer::clearEmergency() {
    emergency_mode_ = false;
}

ControlSurfaces FixedWingMixer::mixElevons(float elevator_cmd, float aileron_cmd, float rudder_cmd, float throttle_cmd) {
    ControlSurfaces controls;
    controls.left_elevon = elevator_cmd + aileron_cmd;
    controls.right_elevon = elevator_cmd - aileron_cmd;
    controls.rudder = rudder_cmd;
    controls.throttle = throttle_cmd;
    return controls;
}

ControlSurfaces FixedWingMixer::mixControls(const AngularRates& controller_outputs,
                                            const RadioInputs& radio_inputs) {
    ControlSurfaces controls;

    const float elevator_command = controller_outputs.pitch_rate * config_.elevator_mix_ratio;
    const float aileron_command = controller_outputs.roll_rate * config_.aileron_mix_ratio;
    const float rudder_command = controller_outputs.yaw_rate * config_.rudder_mix_ratio;

    if (config_.use_elevons) {
        controls.left_elevon = elevator_command + aileron_command;
        controls.right_elevon = elevator_command - aileron_command;

        if (config_.reverse_left_elevon) {
            controls.left_elevon = -controls.left_elevon;
        }
        if (config_.reverse_right_elevon) {
            controls.right_elevon = -controls.right_elevon;
        }
    } else {
        controls.left_elevon = elevator_command;
        controls.right_elevon = aileron_command;
    }

    controls.rudder = config_.reverse_rudder ? -rudder_command : rudder_command;

    if (throttle_passthrough_) {
        controls.throttle = radio_inputs.throttle;
    } else {
        controls.throttle = radio_inputs.throttle;
    }

    return controls;
}

void FixedWingMixer::applySafetyLimits(ControlSurfaces& controls) {
    controls.left_elevon = std::max(-config_.max_elevon_deflection,
                                    std::min(config_.max_elevon_deflection, controls.left_elevon));
    controls.right_elevon = std::max(-config_.max_elevon_deflection,
                                     std::min(config_.max_elevon_deflection, controls.right_elevon));
    controls.rudder = std::max(-config_.max_rudder_deflection,
                               std::min(config_.max_rudder_deflection, controls.rudder));
    controls.throttle = std::max(config_.min_throttle,
                                 std::min(config_.max_throttle, controls.throttle));
}

void FixedWingMixer::applyRateLimiting(ControlSurfaces& controls, float dt) {
    if (dt <= 0.0f) {
        return;
    }

    const float max_rate = 1.0f;
    controls.left_elevon = rateLimitControl(controls.left_elevon, left_elevon_history_, RATE_LIMIT_SAMPLES, max_rate, dt);
    controls.right_elevon = rateLimitControl(controls.right_elevon, right_elevon_history_, RATE_LIMIT_SAMPLES, max_rate, dt);
    controls.rudder = rateLimitControl(controls.rudder, rudder_history_, RATE_LIMIT_SAMPLES, max_rate, dt);

    updateControlHistory(controls.left_elevon, left_elevon_history_);
    updateControlHistory(controls.right_elevon, right_elevon_history_);
    updateControlHistory(controls.rudder, rudder_history_);

    history_index_ = (history_index_ + 1) % RATE_LIMIT_SAMPLES;
}

float FixedWingMixer::rateLimitControl(float new_value, const float* history, int samples, float max_rate, float dt) {
    if (samples <= 0) {
        return new_value;
    }

    const float previous_value = history[(history_index_ - 1 + samples) % samples];
    const float max_change = max_rate * dt;
    const float change = new_value - previous_value;

    if (change > max_change) {
        return previous_value + max_change;
    }
    if (change < -max_change) {
        return previous_value - max_change;
    }
    return new_value;
}

void FixedWingMixer::updateControlHistory(float value, float* history) {
    history[history_index_] = value;
}

void FixedWingMixer::writeActuators(const ControlSurfaces& controls) {
    if (!hal_) {
        return;
    }

    auto normalize = [](float cmd, float trim) {
        return std::max(0.0f, std::min(1.0f, (cmd + trim + 1.0f) * 0.5f));
    };

    hal_->writeServo(left_elevon_channel_, normalize(controls.left_elevon, left_elevon_trim_));
    hal_->writeServo(right_elevon_channel_, normalize(controls.right_elevon, right_elevon_trim_));
    hal_->writeServo(rudder_channel_, normalize(controls.rudder, rudder_trim_));

    const float throttle = std::max(config_.min_throttle,
                                    std::min(config_.max_throttle, controls.throttle));
    hal_->writeMotor(throttle_channel_, throttle);
}
