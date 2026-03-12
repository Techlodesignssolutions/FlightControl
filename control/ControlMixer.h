#pragma once

#include "../core/ControlPipelineTypes.h"

enum class AirframeType {
    Conventional,
    ElevonWing
};

class ControlMixer {
public:
    explicit ControlMixer(AirframeType type) : type_(type) {}

    ActuatorCommands mix(const InnerLoopOutputs& u) const;

private:
    static float toServo01(float symmetric);
    AirframeType type_;
};
