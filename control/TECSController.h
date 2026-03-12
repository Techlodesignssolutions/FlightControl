#pragma once

#include "../core/ControlPipelineTypes.h"

class TECSController {
public:
    bool init() { initialized_ = true; return true; }
    void reset() {}

    TECSOutputs update(const FlightState& state, const OuterLoopCommands& cmd) const;

private:
    bool initialized_{false};
};
