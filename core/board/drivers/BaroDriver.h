#pragma once

class BaroDriver {
public:
    bool init();
    bool readAltitudeMeters(float& altitude_m) const;

    void setAltitudeMeters(float altitude_m);

private:
    float altitude_m_{0.0f};
    bool initialized_{false};
};
