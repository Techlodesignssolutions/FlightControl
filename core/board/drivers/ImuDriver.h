#pragma once

struct ImuSample {
    float gx_rad_s = 0.0f;
    float gy_rad_s = 0.0f;
    float gz_rad_s = 0.0f;
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
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
