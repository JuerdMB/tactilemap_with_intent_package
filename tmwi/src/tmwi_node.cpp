#include <ros/ros.h>
#include "tmwi/Mapper.h"

const std::string PARAM_LOCALIZATION = "localization";


int main(int argc, char **argv) {

    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");

    bool mode_localization;
    if (nodeHandle.getParam(PARAM_LOCALIZATION, mode_localization)){
        ROS_INFO("TMWI_Node running in localization_only mode: %b", mode_localization);
    }
    else{
        mode_localization = false;
        ROS_WARN("Localization param not set, running in standard mapping mode");
    }

    // Create Mapper that will take care of creating transformed, multi-scale maps of environment
    grid_map::GridMap global_map;
    Mapper mapper(nodeHandle, global_map);
    ros::Timer timer = nodeHandle.createTimer(ros::Duration(.5), std::bind(&Mapper::publishTransformedZoomedMap, &mapper));

    if(mode_localization){
        // Check if there is already a database file present
        // Wait for odometry to be found
        // Then start running

        while(!mapper.receivedMap){
            // Try to get map from database
            // global_map = ...
        }
        
    }
    

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
};