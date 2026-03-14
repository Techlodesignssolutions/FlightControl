#pragma once

#include "../types/AircraftState.h"
#include "../types/SensorData.h"

class StateEstimator {
public:
    void update(const SensorData& sensors, AircraftState& state);
};
