// ---------- FlightController.h ----------
#pragma once

#include <cstddef>
#include <cstdint>
#include "HAL.h"
#include "Types.h"
#include "../control/AdaptiveAugmentor.h"
#include "../control/SafetyGovernor.h"
#include "../control/ScheduledLQR.h"
#include "../estimation/StateEstimator.h"
#include "../mixing/FixedWingMixer.h"

/**
 * Main Flight Controller Class
 */
class FlightController {
public:
    struct Config {
        std::uint32_t loop_frequency_hz = 500;
        ControlMode default_mode = ControlMode::STABILIZE;

        float max_attitude_error = 0.785398163f;  // 45 deg in rad
        float radio_timeout_ms = 1000.0f;
        float low_battery_voltage = 10.5f;
        float max_loop_time_ms = 3.0f;
        bool enable_loop_timing_monitoring = true;

        const char* lqr_schedule_path = nullptr;  // must be absolute

        bool use_pitot_airspeed_primary = true;

        bool enable_debug_output = true;
        std::uint32_t debug_output_rate_hz = 10;
    };

    enum class Status {
        INITIALIZING,
        READY,
        ARMED,
        FLYING,
        EMERGENCY,
        SYSTEM_ERROR
    };

    // Constructor / Lifecycle
    FlightController(HAL* hal, const Config& config = Config());
    bool initialize();
    void update();

    // Accessors
    Status getStatus() const { return status_; }
    AircraftState getAircraftState() const;
    ControlMode getControlMode() const { return control_mode_; }
    PerformanceStats getPerformanceStats() const { return performance_stats_; }
    DiagnosticInfo getDiagnosticInfo() const;

    // Control
    bool setControlMode(ControlMode mode);
    void emergencyStop();
    bool arm();          // Arm the flight controller with safety checks
    bool disarm();       // Disarm the flight controller
    bool isArmed() const { return armed_; }

    // Config persistence
    bool loadConfiguration(const void* data, std::size_t size);
    bool saveConfiguration(void* data, std::size_t* size) const;

private:
    // Hardware abstraction
    HAL* hal_;
    Config config_;

    // State
    Status status_;
    ControlMode control_mode_;

    // Subsystems
    StateEstimator state_estimator_;
    ScheduledLQR scheduled_lqr_;
    AdaptiveAugmentor adaptive_augmentor_;
    SafetyGovernor safety_governor_;
    FixedWingMixer mixer_;

    // Flight data
    RadioInputs radio_inputs_;
    RadioInputs previous_radio_inputs_;
    AircraftState aircraft_state_;
    ControlSurfaces control_outputs_;
    AngularRates control_commands_;

    // Timing
    std::uint32_t loop_start_time_;
    std::uint32_t last_loop_time_;
    float loop_dt_;
    std::uint32_t last_mode_change_ms_;


    // Safety & performance
    bool armed_{false};
    bool emergency_mode_{false};
    bool control_stack_ready_{false};
    bool last_learning_enabled_{false};
    bool last_saturation_{false};
    float previous_airspeed_{0.0f};
    float last_r_cmd_{0.0f};

    static constexpr int TIMING_SAMPLES = 100;
    PerformanceStats performance_stats_;
    float loop_time_history_[TIMING_SAMPLES]{};
    int timing_index_{0};

    // Failsafe monitoring
    std::uint32_t last_radio_update_{0};
    std::uint32_t radio_timeout_start_{0};
    bool radio_timeout_active_{false};
    float battery_voltage_{0.0f};

    // Debug output
    std::uint32_t last_debug_output_{0};

    // Core steps
    bool readRadioInputs();
    bool updateStateEstimation();
    bool runControlLoops();
    bool mixAndOutputControls();
    void updateSafetyMonitoring();
    void updatePerformanceStats();
    void outputDebugInfo();

    // Helpers
    bool checkSystemHealth();
    void handleRadioTimeout();
    void handleBatteryLow();
    bool isInEmergencyCondition() const;

    bool initializeControlStack();

    void runManualMode();
    void runClosedLoopAttitudeMode();
    void runStabilizeMode();
    void runAltitudeMode();
    void runPositionMode();
    void runAutoMode();

    bool shouldEnableLearning(const SafetyGovernor::Result& governor_result, float stick_step_mag) const;
    float calculateStickStepMagnitude() const;

    void updateLoopTimingStats(float loop_time_ms);
    bool isRadioSignalValid() const;
    float readBatteryVoltage() const;
    bool readMeasuredAirspeed(float& airspeed_mps) const;
};



