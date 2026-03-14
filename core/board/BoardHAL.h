#pragma once

#include "../types/ActuatorCommand.h"
#include "../types/PilotInput.h"
#include "../types/SensorData.h"

class BoardHAL {
public:
    virtual ~BoardHAL() = default;

    virtual bool init() = 0;
    virtual bool readSensors(SensorData& sensor_data) = 0;
    virtual bool readPilotInput(PilotInput& pilot_input) = 0;
    virtual bool writeActuators(const ActuatorCommand& cmd) = 0;
    virtual unsigned long microsNow() = 0;
};
