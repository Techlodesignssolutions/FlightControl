#include "TECSController.h"

TECSOutputs TECSController::update(const FlightState&, const OuterLoopCommands&) const {
    TECSOutputs out;
    out.valid = false;
    out.pitch_cmd_rad = 0.0f;
    out.throttle_cmd_0to1 = 0.0f;
    return out;
}
