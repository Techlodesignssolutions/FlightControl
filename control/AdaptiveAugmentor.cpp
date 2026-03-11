#include "AdaptiveAugmentor.h"

#include <algorithm>

void AdaptiveAugmentor::configure(const Config& config) {
    config_ = config;
}

float AdaptiveAugmentor::dot3(const std::array<float, 3>& w, const std::array<float, 3>& phi) {
    return w[0] * phi[0] + w[1] * phi[1] + w[2] * phi[2];
}

float AdaptiveAugmentor::clamp(float value, float limit) const {
    return std::max(-limit, std::min(limit, value));
}

AdaptiveAugmentor::Output AdaptiveAugmentor::compute(float e_phi, float p,
                                                     float e_theta, float q,
                                                     float e_r, float r) const {
    const std::array<float, 3> phi_roll = { 1.0f, e_phi, p };
    const std::array<float, 3> phi_pitch = { 1.0f, e_theta, q };
    const std::array<float, 3> phi_yaw = { 1.0f, e_r, r };

    Output out;
    out.ua = clamp(dot3(w_roll_, phi_roll), config_.output_limit_roll);
    out.ue = clamp(dot3(w_pitch_, phi_pitch), config_.output_limit_pitch);
    out.ur = clamp(dot3(w_yaw_, phi_yaw), config_.output_limit_yaw);
    return out;
}

void AdaptiveAugmentor::updateAxis(std::array<float, 3>& w,
                                   const std::array<float, 3>& phi,
                                   float error,
                                   float dt) {
    for (int i = 0; i < 3; ++i) {
        const float delta = (config_.learning_rate * error * phi[i] - config_.leakage * w[i]) * dt;
        w[i] += delta;
        w[i] = clamp(w[i], config_.weight_limit);
    }
}

void AdaptiveAugmentor::update(float dt,
                               float e_phi, float p,
                               float e_theta, float q,
                               float e_r, float r,
                               bool learning_enabled) {
    learning_active_ = false;

    if (frozen_ || !learning_enabled || dt <= 0.0f) {
        return;
    }

    const std::array<float, 3> phi_roll = { 1.0f, e_phi, p };
    const std::array<float, 3> phi_pitch = { 1.0f, e_theta, q };
    const std::array<float, 3> phi_yaw = { 1.0f, e_r, r };

    updateAxis(w_roll_, phi_roll, e_phi, dt);
    updateAxis(w_pitch_, phi_pitch, e_theta, dt);
    updateAxis(w_yaw_, phi_yaw, e_r, dt);

    learning_active_ = true;
}

void AdaptiveAugmentor::reset() {
    w_roll_ = { 0.0f, 0.0f, 0.0f };
    w_pitch_ = { 0.0f, 0.0f, 0.0f };
    w_yaw_ = { 0.0f, 0.0f, 0.0f };
    learning_active_ = false;
}
