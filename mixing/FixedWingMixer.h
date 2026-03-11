#pragma once

#include "../core/HAL.h"
#include "../core/Types.h"

/**
 * Fixed-Wing Control Mixer
 *
 * Converts controller axis commands into actuator commands for fixed-wing elevon aircraft.
 * Mixer responsibilities are intentionally minimal:
 * - elevon geometry
 * - control reversals
 * - actuator limits
 * - output slew limiting
 */
class FixedWingMixer {
public:
    struct Config {
        // Geometry and direction
        bool use_elevons = true;
        bool reverse_left_elevon = false;
        bool reverse_right_elevon = false;
        bool reverse_rudder = false;

        // Authority limits
        float max_elevon_deflection = 0.8f;
        float max_rudder_deflection = 0.6f;
        float max_throttle = 1.0f;
        float min_throttle = 0.0f;

        // Mixing ratios
        float elevator_mix_ratio = 1.0f;
        float aileron_mix_ratio = 1.0f;
        float rudder_mix_ratio = 1.0f;

        // Throttle management
        float idle_throttle = 0.1f;

        // Hardware mapping
        int left_elevon_channel = 0;
        int right_elevon_channel = 1;
        int rudder_channel = 2;
        int motor_channel = 0;
    };

    FixedWingMixer(HAL* hal, const Config& config = Config());

    bool initialize();

    ControlSurfaces mix(const AngularRates& controller_outputs,
                        const RadioInputs& radio_inputs);

    void applyControls(const ControlSurfaces& controls);

    void emergencyStop();
    void clearEmergency();

    ControlSurfaces getCurrentControls() const { return current_controls_; }
    void setThrottlePassthrough(bool enable) { throttle_passthrough_ = enable; }

    static ControlSurfaces mixElevons(float elevator_cmd, float aileron_cmd, float rudder_cmd, float throttle_cmd);

private:
    HAL* hal_;
    Config config_;
    ControlSurfaces current_controls_;

    bool initialized_;
    bool throttle_passthrough_;
    bool emergency_mode_;

    int left_elevon_channel_;
    int right_elevon_channel_;
    int rudder_channel_;
    int throttle_channel_;

    float left_elevon_trim_;
    float right_elevon_trim_;
    float rudder_trim_;

    static const int RATE_LIMIT_SAMPLES = 5;
    float left_elevon_history_[RATE_LIMIT_SAMPLES];
    float right_elevon_history_[RATE_LIMIT_SAMPLES];
    float rudder_history_[RATE_LIMIT_SAMPLES];
    int history_index_;

    uint32_t last_update_time_;

    ControlSurfaces mixControls(const AngularRates& controller_outputs, const RadioInputs& radio_inputs);
    void applySafetyLimits(ControlSurfaces& controls);
    void applyRateLimiting(ControlSurfaces& controls, float dt);

    float rateLimitControl(float new_value, const float* history, int samples, float max_rate, float dt);
    void updateControlHistory(float value, float* history);

    void writeActuators(const ControlSurfaces& controls);
};
