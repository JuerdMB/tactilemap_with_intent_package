#pragma once

#include <ros/ros.h>
#include <vector>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

class MapDataConverter
{

public:
    MapDataConverter();

    static std::vector<uint8_t> mapToDataArray(grid_map::GridMap &inputMap, const std::string &layer);
    static void mapToScreenResolution(grid_map::GridMap &inputMap, grid_map::GridMap &outputMap);

};