#pragma once

class PitotDriver {
public:
    bool init();
    bool readDifferentialPressurePa(float& dp_pa) const;

    void setDifferentialPressurePa(float dp_pa);

private:
    float dp_pa_{0.0f};
    bool initialized_{false};
};
