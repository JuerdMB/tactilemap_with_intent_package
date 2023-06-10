#include <ros/ros.h>
#include "tactilemap_with_intent_package/Mapper.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");

    // Create Mapper that will take care of creating custom, transformed, zoomed maps
    grid_map::GridMap global_map;
    Mapper mapper(nodeHandle, global_map);

    // Every 2 seconds, try to send the map to the screen
    ros::Timer mapdata_transfer_timer = nodeHandle.createTimer(ros::Duration(4.), std::bind(&Mapper::publishTransformedZoomedMap, &mapper));

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
};