#pragma once

#include "../board/BoardHAL.h"
#include "../control/AttitudeController.h"
#include "../mixing/Mixer.h"
#include "../state/StateEstimator.h"

class FlightController {
public:
    explicit FlightController(BoardHAL& hal);

    bool init(const LQRScheduleTable& table);
    void update();

private:
    BoardHAL& hal_;
    StateEstimator estimator_;
    AttitudeController controller_;
    Mixer mixer_;

    unsigned long last_time_us_ = 0;
};
