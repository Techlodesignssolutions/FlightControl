#include "StateEstimator.h"

void StateEstimator::update(const SensorData& sensors, AircraftState& state) {
    state.roll_rad = sensors.roll_rad;
    state.pitch_rad = sensors.pitch_rad;
    state.yaw_rad = sensors.yaw_rad;

    state.p_rad_s = sensors.p_rad_s;
    state.q_rad_s = sensors.q_rad_s;
    state.r_rad_s = sensors.r_rad_s;

    state.altitude_m = sensors.altitude_m;
    state.climb_rate_mps = sensors.climb_rate_mps;
    state.airspeed_mps = sensors.airspeed_mps;
}
