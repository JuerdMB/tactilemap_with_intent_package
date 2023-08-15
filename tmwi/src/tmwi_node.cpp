#include <ros/ros.h>
#include "tmwi/Mapper.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");

    // Create Mapper that will take care of creating transformed, multi-scale maps of environment
    grid_map::GridMap global_map;
    Mapper mapper(nodeHandle, global_map);
    ros::Timer timer = nodeHandle.createTimer(ros::Duration(.5), std::bind(&Mapper::publishTransformedZoomedMap, &mapper));

    // Check if there is already a database file present
    // Wait for odometry to be found
    // Then start running

    // if(mode_localization) ask db file to publish as soon as odometry is found

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
};