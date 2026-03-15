#pragma once

#include <cstdint>

struct SensorData {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    float p = 0.0f;
    float q = 0.0f;
    float r = 0.0f;

    float altitude = 0.0f;
    float climb_rate = 0.0f;
    float pitot_airspeed = 0.0f;
};

struct PilotInput {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;
    bool armed = false;
};

struct ActuatorCommand {
    float left_elevon = 0.0f;   // [-1, 1]
    float right_elevon = 0.0f;  // [-1, 1]
    float rudder = 0.0f;        // [-1, 1]
    float throttle = 0.0f;      // [0, 1]
};

class BoardHAL {
public:
    virtual ~BoardHAL() = default;

    virtual bool init() = 0;
    virtual bool readSensors(SensorData& sensor_data) = 0;
    virtual bool readPilotInput(PilotInput& pilot_input) = 0;
    virtual bool writeActuators(const ActuatorCommand& cmd) = 0;
    virtual unsigned long microsNow() = 0;
};
