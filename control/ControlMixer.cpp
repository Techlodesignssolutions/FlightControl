#include "ControlMixer.h"

#include <algorithm>

ActuatorCommands ControlMixer::mix(const InnerLoopOutputs& u) const {
    ActuatorCommands act;

    if (type_ == AirframeType::ElevonWing) {
        const float left = u.pitch_surface_cmd + u.roll_surface_cmd;
        const float right = u.pitch_surface_cmd - u.roll_surface_cmd;
        act.left_surface_0to1 = toServo01(left);
        act.right_surface_0to1 = toServo01(right);
    } else {
        act.left_surface_0to1 = toServo01(u.roll_surface_cmd);
        act.right_surface_0to1 = toServo01(-u.roll_surface_cmd);
    }

    act.rudder_0to1 = toServo01(u.yaw_surface_cmd);
    act.throttle_0to1 = std::max(0.0f, std::min(1.0f, u.throttle_cmd_0to1));
    return act;
}

float ControlMixer::toServo01(float symmetric) {
    const float clipped = std::max(-1.0f, std::min(1.0f, symmetric));
    return 0.5f * (clipped + 1.0f);
}
