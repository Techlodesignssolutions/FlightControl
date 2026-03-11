#pragma once

#include <array>

class AdaptiveAugmentor {
public:
    struct Config {
        float learning_rate = 0.01f;
        float leakage = 0.01f;
        float weight_limit = 0.25f;
        float output_limit_roll = 0.25f;
        float output_limit_pitch = 0.25f;
        float output_limit_yaw = 0.25f;
    };

    struct Output {
        float ua = 0.0f;
        float ue = 0.0f;
        float ur = 0.0f;
    };

    AdaptiveAugmentor() = default;
    explicit AdaptiveAugmentor(const Config& config) : config_(config) {}

    void configure(const Config& config);

    Output compute(float e_phi, float p,
                   float e_theta, float q,
                   float e_r, float r) const;

    void update(float dt,
                float e_phi, float p,
                float e_theta, float q,
                float e_r, float r,
                bool learning_enabled);

    void reset();
    void freeze(bool on) { frozen_ = on; }

    bool isLearning() const { return learning_active_; }

    const std::array<float, 3>& rollWeights() const { return w_roll_; }
    const std::array<float, 3>& pitchWeights() const { return w_pitch_; }
    const std::array<float, 3>& yawWeights() const { return w_yaw_; }

private:
    static float dot3(const std::array<float, 3>& w, const std::array<float, 3>& phi);
    float clamp(float value, float limit) const;
    void updateAxis(std::array<float, 3>& w,
                    const std::array<float, 3>& phi,
                    float error,
                    float dt);

    Config config_{};

    std::array<float, 3> w_roll_{ {0.0f, 0.0f, 0.0f} };
    std::array<float, 3> w_pitch_{ {0.0f, 0.0f, 0.0f} };
    std::array<float, 3> w_yaw_{ {0.0f, 0.0f, 0.0f} };

    bool frozen_ = false;
    bool learning_active_ = false;
};
