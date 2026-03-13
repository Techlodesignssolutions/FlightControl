#pragma once

struct ImuSample {
    float gx_rad_s = 0.0f;
    float gy_rad_s = 0.0f;
    float gz_rad_s = 0.0f;

    float ax_m_s2 = 0.0f;
    float ay_m_s2 = 0.0f;
    float az_m_s2 = 9.80665f;
};

class ImuDriver {
public:
    bool init();
    bool read(ImuSample& out) const;

    void setSample(const ImuSample& sample);

private:
    ImuSample sample_{};
    bool initialized_{false};
};
