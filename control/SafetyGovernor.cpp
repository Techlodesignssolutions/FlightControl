#include "SafetyGovernor.h"

#include <algorithm>
#include <cmath>

float SafetyGovernor::clamp(float value, float lo, float hi) {
    return std::max(lo, std::min(hi, value));
}

float SafetyGovernor::rateLimit(float target, float previous, float max_rate, float dt) {
    if (dt <= 0.0f || max_rate <= 0.0f) {
        return target;
    }

    const float max_delta = max_rate * dt;
    const float delta = target - previous;
    if (delta > max_delta) {
        return previous + max_delta;
    }
    if (delta < -max_delta) {
        return previous - max_delta;
    }
    return target;
}

void SafetyGovernor::reset() {
    previous_adaptive_ = AxisValues{};
}

SafetyGovernor::Result SafetyGovernor::apply(float dt,
                                             const AxisValues& base,
                                             const AxisValues& adaptive_request) {
    Result result;
    result.base = base;

    auto process_axis = [&](float requested, float prev, float adaptive_limit, float adaptive_rate_limit, float total_limit,
                            float base_cmd, float& adaptive_out, float& total_out) {
        const float clamped_adaptive = clamp(requested, -adaptive_limit, adaptive_limit);
        if (std::fabs(clamped_adaptive - requested) > 1e-6f) {
            result.adaptive_saturated = true;
        }

        const float rate_limited_adaptive = rateLimit(clamped_adaptive, prev, adaptive_rate_limit, dt);
        if (std::fabs(rate_limited_adaptive - clamped_adaptive) > 1e-6f) {
            result.adaptive_saturated = true;
        }

        adaptive_out = rate_limited_adaptive;

        const float requested_total = base_cmd + rate_limited_adaptive;
        total_out = clamp(requested_total, -total_limit, total_limit);
        if (std::fabs(total_out - requested_total) > 1e-6f) {
            result.total_saturated = true;
        }
    };

    process_axis(adaptive_request.roll, previous_adaptive_.roll,
                 config_.adaptive_limit_roll, config_.adaptive_rate_limit_roll, config_.total_limit_roll,
                 base.roll, result.adaptive.roll, result.total.roll);

    process_axis(adaptive_request.pitch, previous_adaptive_.pitch,
                 config_.adaptive_limit_pitch, config_.adaptive_rate_limit_pitch, config_.total_limit_pitch,
                 base.pitch, result.adaptive.pitch, result.total.pitch);

    process_axis(adaptive_request.yaw, previous_adaptive_.yaw,
                 config_.adaptive_limit_yaw, config_.adaptive_rate_limit_yaw, config_.total_limit_yaw,
                 base.yaw, result.adaptive.yaw, result.total.yaw);

    previous_adaptive_ = result.adaptive;
    return result;
}
