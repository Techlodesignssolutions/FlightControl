#pragma once

#include "LQRScheduleTable.h"

#include <string>
#include <vector>

class ScheduleTableLoader {
public:
    bool loadFromJsonFile(const char* path, std::vector<LQRGainPoint>& out_points, std::string* error = nullptr) const;
    bool buildTable(std::vector<LQRGainPoint>& points, LQRScheduleTable& out_table) const;
};
