#pragma once

class PitotDriver {
public:
    enum class Source {
        DigitalI2C,
        AnalogAir
    };

    bool init(Source source);
    bool readDifferentialPressurePa(float& dp_pa) const;

    void setDifferentialPressurePa(float dp_pa);

private:
    Source source_{Source::DigitalI2C};
    float dp_pa_{0.0f};
    bool initialized_{false};
};
