#pragma once

#include "../types/ActuatorCommand.h"
#include "../types/ControlEffort.h"

class Mixer {
public:
    ActuatorCommand mix(const ControlEffort& effort) const;
};
