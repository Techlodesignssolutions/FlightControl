#pragma once

class SafetyGovernor {
public:
    struct Config {
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

    struct AxisValues {
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
    };

    struct Result {
        AxisValues base;
        AxisValues adaptive;
        AxisValues total;
        bool adaptive_saturated = false;
        bool total_saturated = false;
    };

    SafetyGovernor() = default;
    explicit SafetyGovernor(const Config& config) : config_(config) {}

    void configure(const Config& config) { config_ = config; }
    void reset();

    Result apply(float dt, const AxisValues& base, const AxisValues& adaptive_request);

private:
    static float clamp(float value, float lo, float hi);
    static float rateLimit(float target, float previous, float max_rate, float dt);

    Config config_{};
    AxisValues previous_adaptive_{};
};
