#include "Mixer.h"

namespace {

float clamp1(float x) {
    if (x > 1.0f) {
        return 1.0f;
    }
    if (x < -1.0f) {
        return -1.0f;
    }
    return x;
}

float clampThrottle(float x) {
    if (x > 1.0f) {
        return 1.0f;
    }
    if (x < 0.0f) {
        return 0.0f;
    }
    return x;
}

}  // namespace

ActuatorCommand Mixer::mix(const ControlEffort& effort) const {
    ActuatorCommand cmd{};
    cmd.left_elevon = clamp1(effort.pitch + effort.roll);
    cmd.right_elevon = clamp1(effort.pitch - effort.roll);
    cmd.rudder = clamp1(effort.yaw);
    cmd.throttle = clampThrottle(effort.throttle);
    return cmd;
}
