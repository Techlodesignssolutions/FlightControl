#pragma once

#include "GainProvider.h"
#include "ScheduledLQR.h"

class ScheduledGainProvider final : public GainProvider {
public:
    ScheduledGainProvider() = default;

    bool configure(const ScheduledLQR::ScheduleConfig& schedule, const ScheduledLQR::ControllerParams& controller);
    bool isConfigured() const { return configured_; }

    AxisGains2State getRollGains(float airspeed_mps) const override;
    AxisGains2State getPitchGains(float airspeed_mps) const override;
    AxisGains2State getYawGains(float airspeed_mps) const override;

    float yawStickGain() const { return yaw_stick_gain_; }
    float maxRollCommandRad() const { return max_roll_cmd_rad_; }
    float maxPitchCommandRad() const { return max_pitch_cmd_rad_; }
    float yawCoordinationGain(float airspeed_mps) const;

private:
    AxisGains2State interpolateAxis(float airspeed_mps, ScheduledLQR::AxisPoint ScheduledLQR::SchedulePoint::*member) const;

    ScheduledLQR::ScheduleConfig schedule_{};
    bool configured_{false};
    float yaw_stick_gain_{1.0f};
    float max_roll_cmd_rad_{0.7f};
    float max_pitch_cmd_rad_{0.5f};
};
