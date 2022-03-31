#pragma once

#include <string>
#include <vector>

struct cam_data {
    double time;
    std::string path;
};

using cam_dataset = std::pair<std::vector<cam_data>, std::vector<cam_data>>;
