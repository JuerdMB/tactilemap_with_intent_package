#include <ros/ros.h>
#include "tmwi/Mapper.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");

    // Create Mapper that will take care of creating transformed, multi-scale maps of environment
    grid_map::GridMap global_map;
    Mapper mapper(nodeHandle, global_map);

    ros::Timer timer = nodeHandle.createTimer(ros::Duration(.2), std::bind(&Mapper::publishTransformedZoomedMap, &mapper));

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
};