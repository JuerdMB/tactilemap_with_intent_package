#include <ros/ros.h>

#include "tactilemap_with_intent_package/Mapper.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");

    // Create Mapper that will take care of creating custom, transformed, zoomed maps
    grid_map::GridMap fullMap(basicLayers);
    grid_map::GridMap transformedMap(basicLayers);
    Mapper mapper(nodeHandle, fullMap, transformedMap);

    // The mapper will start listening for maps from ROS

    // Every .5 seconds, transform the map and 
    ros::Timer map_update_timer = nodeHandle.createTimer(ros::Duration(.5), std::bind(&Mapper::updateTransformedMap, &mapper));

    // Every 2 seconds, try to send the map to the screen
    ros::Timer mapdata_transfer_timer = nodeHandle.createTimer(ros::Duration(2.), std::bind(&Mapper::publishMap, &mapper));

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
};