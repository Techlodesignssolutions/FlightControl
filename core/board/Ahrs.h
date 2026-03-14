#pragma once

class Ahrs {
public:
    void reset(float roll_rad, float pitch_rad, float yaw_rad);
    void update(float gx_rad_s,
                float gy_rad_s,
                float gz_rad_s,
                float ax_m_s2,
                float ay_m_s2,
                float az_m_s2,
                float dt_s,
                float alpha);

    float roll() const { return roll_rad_; }
    float pitch() const { return pitch_rad_; }
    float yaw() const { return yaw_rad_; }

private:
    float roll_rad_{0.0f};
    float pitch_rad_{0.0f};
    float yaw_rad_{0.0f};
};
