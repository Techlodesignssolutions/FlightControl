#include "ScheduledLQR.h"

bool ScheduledLQR::init(const LQRScheduleTable& table) {
    table_ = table;
    return (table_.points != nullptr && table_.count > 0);
}

LQRGainPoint ScheduledLQR::interpolate(float airspeed_mps) const {
    if (table_.count == 1) {
        return table_.points[0];
    }

    if (airspeed_mps <= table_.points[0].airspeed_mps) {
        return table_.points[0];
    }

    for (std::size_t i = 0; i < table_.count - 1; ++i) {
        const LQRGainPoint& a = table_.points[i];
        const LQRGainPoint& b = table_.points[i + 1];

        if (airspeed_mps >= a.airspeed_mps && airspeed_mps <= b.airspeed_mps) {
            const float t = (airspeed_mps - a.airspeed_mps) / (b.airspeed_mps - a.airspeed_mps);

            LQRGainPoint out{};
            out.airspeed_mps = airspeed_mps;
            out.k_phi = a.k_phi + t * (b.k_phi - a.k_phi);
            out.k_p = a.k_p + t * (b.k_p - a.k_p);
            out.k_theta = a.k_theta + t * (b.k_theta - a.k_theta);
            out.k_q = a.k_q + t * (b.k_q - a.k_q);
            return out;
        }
    }

    return table_.points[table_.count - 1];
}

ControlEffort ScheduledLQR::compute(const AircraftState& state, const AttitudeSetpoint& setpoint) const {
    const LQRGainPoint gains = interpolate(state.airspeed_mps);

    const float phi_error = state.roll_rad - setpoint.roll_rad;
    const float theta_error = state.pitch_rad - setpoint.pitch_rad;

    ControlEffort u{};
    u.roll = -(gains.k_phi * phi_error + gains.k_p * state.p_rad_s);
    u.pitch = -(gains.k_theta * theta_error + gains.k_q * state.q_rad_s);
    u.yaw = setpoint.yaw_cmd;
    u.throttle = setpoint.throttle_cmd;
    return u;
}
