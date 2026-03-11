#pragma once

#include <cstddef>
#include <string>
#include <vector>

class ScheduledLQR {
public:
    struct AxisPoint {
        float k_error = 0.0f;
        float k_rate = 0.0f;
        float trim = 0.0f;
    };

    struct SchedulePoint {
        float airspeed = 0.0f;
        AxisPoint roll;
        AxisPoint pitch;
        AxisPoint yaw;
        float yaw_coord_gain = 0.0f;
    };

    struct AdaptiveParams {
        float learning_rate = 0.01f;
        float leakage = 0.01f;
        float weight_limit = 0.25f;
        float output_limit_roll = 0.25f;
        float output_limit_pitch = 0.25f;
        float output_limit_yaw = 0.25f;
        float near_stall_airspeed = 4.0f;
        float mode_change_freeze_s = 1.0f;
        float stick_step_threshold = 0.2f;
        float regime_stability_delta = 1.0f;
    };

    struct SafetyParams {
        float adaptive_limit_roll = 0.25f;
        float adaptive_limit_pitch = 0.25f;
        float adaptive_limit_yaw = 0.25f;

        float adaptive_rate_limit_roll = 1.0f;
        float adaptive_rate_limit_pitch = 1.0f;
        float adaptive_rate_limit_yaw = 1.0f;

        float total_limit_roll = 1.0f;
        float total_limit_pitch = 1.0f;
        float total_limit_yaw = 1.0f;
    };

    struct ControllerParams {
        float yaw_stick_gain = 1.0f;
        float max_roll_cmd_rad = 0.785398163f;   // 45 deg
        float max_pitch_cmd_rad = 0.523598775f;  // 30 deg
    };

    struct ScheduleConfig {
        int version = 0;
        std::string name;
        std::vector<SchedulePoint> points;
        AdaptiveParams adaptive;
        SafetyParams safety;
        ControllerParams controller;
    };

    bool loadFromJsonFile(const char* absolute_path, ScheduleConfig& out_config, std::string* error_message = nullptr) const;
    bool configure(const ScheduleConfig& config, std::string* error_message = nullptr);

    bool isConfigured() const { return configured_; }

    float computeRoll(float airspeed, float phi, float p, float phi_cmd) const;
    float computePitch(float airspeed, float theta, float q, float theta_cmd) const;
    float computeYaw(float airspeed, float r, float r_cmd) const;

    float interpolateYawCoordinationGain(float airspeed) const;

    float yawStickGain() const { return config_.controller.yaw_stick_gain; }
    float maxRollCommandRad() const { return config_.controller.max_roll_cmd_rad; }
    float maxPitchCommandRad() const { return config_.controller.max_pitch_cmd_rad; }

    const AdaptiveParams& adaptiveParams() const { return config_.adaptive; }
    const SafetyParams& safetyParams() const { return config_.safety; }
    const ScheduleConfig& config() const { return config_; }

private:
    struct InterpIndex {
        std::size_t low = 0;
        std::size_t high = 0;
        float t = 0.0f;
    };

    static bool isAbsolutePath(const std::string& path);
    static float clamp(float value, float lo, float hi);

    InterpIndex getInterpolation(float airspeed) const;
    float interpolateAxis(float airspeed, AxisPoint SchedulePoint::*axis_member, float error, float rate) const;

    ScheduleConfig config_{};
    bool configured_ = false;
};

