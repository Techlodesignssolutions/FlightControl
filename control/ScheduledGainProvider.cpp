#include "ScheduledGainProvider.h"

#include <algorithm>

namespace {

float clampf(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}

float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

}  // namespace

bool ScheduledGainProvider::configure(const ScheduledLQR::ScheduleConfig& schedule,
                                      const ScheduledLQR::ControllerParams& controller) {
    if (schedule.points.size() < 2) {
        configured_ = false;
        return false;
    }

    schedule_ = schedule;
    yaw_stick_gain_ = controller.yaw_stick_gain;
    max_roll_cmd_rad_ = controller.max_roll_cmd_rad;
    max_pitch_cmd_rad_ = controller.max_pitch_cmd_rad;
    configured_ = true;
    return true;
}

AxisGains2State ScheduledGainProvider::interpolateAxis(float airspeed_mps,
                                                       ScheduledLQR::AxisPoint ScheduledLQR::SchedulePoint::*member) const {
    AxisGains2State gains;
    if (!configured_ || schedule_.points.empty()) {
        return gains;
    }

    const auto& points = schedule_.points;
    if (airspeed_mps <= points.front().airspeed) {
        gains.k1 = (points.front().*member).k_error;
        gains.k2 = (points.front().*member).k_rate;
        gains.trim = (points.front().*member).trim;
        return gains;
    }

    if (airspeed_mps >= points.back().airspeed) {
        gains.k1 = (points.back().*member).k_error;
        gains.k2 = (points.back().*member).k_rate;
        gains.trim = (points.back().*member).trim;
        return gains;
    }

    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const float v0 = points[i].airspeed;
        const float v1 = points[i + 1].airspeed;
        if (airspeed_mps < v0 || airspeed_mps > v1) {
            continue;
        }

        const float t = clampf((airspeed_mps - v0) / std::max(1e-6f, v1 - v0), 0.0f, 1.0f);
        const auto& a0 = points[i].*member;
        const auto& a1 = points[i + 1].*member;
        gains.k1 = lerp(a0.k_error, a1.k_error, t);
        gains.k2 = lerp(a0.k_rate, a1.k_rate, t);
        gains.trim = lerp(a0.trim, a1.trim, t);
        return gains;
    }

    return gains;
}

AxisGains2State ScheduledGainProvider::getRollGains(float airspeed_mps) const {
    return interpolateAxis(airspeed_mps, &ScheduledLQR::SchedulePoint::roll);
}

AxisGains2State ScheduledGainProvider::getPitchGains(float airspeed_mps) const {
    return interpolateAxis(airspeed_mps, &ScheduledLQR::SchedulePoint::pitch);
}

AxisGains2State ScheduledGainProvider::getYawGains(float airspeed_mps) const {
    return interpolateAxis(airspeed_mps, &ScheduledLQR::SchedulePoint::yaw);
}

float ScheduledGainProvider::yawCoordinationGain(float airspeed_mps) const {
    if (!configured_ || schedule_.points.empty()) {
        return 0.0f;
    }

    const auto& points = schedule_.points;
    if (airspeed_mps <= points.front().airspeed) {
        return points.front().yaw_coord_gain;
    }
    if (airspeed_mps >= points.back().airspeed) {
        return points.back().yaw_coord_gain;
    }

    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const float v0 = points[i].airspeed;
        const float v1 = points[i + 1].airspeed;
        if (airspeed_mps < v0 || airspeed_mps > v1) {
            continue;
        }

        const float t = clampf((airspeed_mps - v0) / std::max(1e-6f, v1 - v0), 0.0f, 1.0f);
        return lerp(points[i].yaw_coord_gain, points[i + 1].yaw_coord_gain, t);
    }

    return points.back().yaw_coord_gain;
}
