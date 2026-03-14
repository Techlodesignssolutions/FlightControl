#pragma once

#include <vector>

#include "../board/BoardHAL.h"
#include "../control/AttitudeController.h"
#include "../control/ScheduleTableLoader.h"
#include "../mixing/Mixer.h"
#include "../state/StateEstimator.h"

class FlightController {
public:
    explicit FlightController(BoardHAL& hal);

    bool init(const LQRScheduleTable& table);
    bool initFromScheduleJson(const char* schedule_json_path);
    void update();

private:
    BoardHAL& hal_;
    StateEstimator estimator_;
    AttitudeController controller_;
    Mixer mixer_;

    std::vector<LQRGainPoint> owned_schedule_points_{};
    ScheduleTableLoader schedule_loader_{};

    unsigned long last_time_us_ = 0;
};
