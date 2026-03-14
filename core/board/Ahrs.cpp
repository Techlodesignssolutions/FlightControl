#include "Ahrs.h"

#include <algorithm>
#include <cmath>

void Ahrs::reset(float roll_rad, float pitch_rad, float yaw_rad) {
    roll_rad_ = roll_rad;
    pitch_rad_ = pitch_rad;
    yaw_rad_ = yaw_rad;
}

void Ahrs::update(float gx_rad_s,
                  float gy_rad_s,
                  float gz_rad_s,
                  float ax_m_s2,
                  float ay_m_s2,
                  float az_m_s2,
                  float dt_s,
                  float alpha) {
    const float a = std::max(0.0f, std::min(1.0f, alpha));

    const float roll_acc = std::atan2(ay_m_s2, az_m_s2);
    const float pitch_acc = std::atan2(-ax_m_s2, std::sqrt(ay_m_s2 * ay_m_s2 + az_m_s2 * az_m_s2));

    roll_rad_ = a * (roll_rad_ + gx_rad_s * dt_s) + (1.0f - a) * roll_acc;
    pitch_rad_ = a * (pitch_rad_ + gy_rad_s * dt_s) + (1.0f - a) * pitch_acc;
    yaw_rad_ += gz_rad_s * dt_s;
}
