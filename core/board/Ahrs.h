#pragma once

#include <cmath>

class Ahrs {
public:
    void reset() {
        roll_ = 0.0f;
        pitch_ = 0.0f;
        yaw_ = 0.0f;
    }

    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt_s, float alpha) {
        if (dt_s <= 0.0f) {
            return;
        }

        roll_ += gx * dt_s;
        pitch_ += gy * dt_s;
        yaw_ += gz * dt_s;

        const float accel_roll = std::atan2(ay, az);
        const float accel_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

        roll_ = alpha * roll_ + (1.0f - alpha) * accel_roll;
        pitch_ = alpha * pitch_ + (1.0f - alpha) * accel_pitch;
    }

    float roll() const { return roll_; }
    float pitch() const { return pitch_; }
    float yaw() const { return yaw_; }

private:
    float roll_ = 0.0f;
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
};
