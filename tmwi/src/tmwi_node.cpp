#include <ros/ros.h>
#include "tmwi/Mapper.h"

const std::string PARAM_LOCALIZATION = "rtabmap/localization";
const std::string PARAM_MAPRATE = "tmwi/map_freq";
const std::string PARAM_MAP_MODE = "tmwi/map_mode";



int main(int argc, char **argv) {

    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");

    bool mode_localization;
    if (nodeHandle.getParam(PARAM_LOCALIZATION, mode_localization)){
        ROS_INFO("TMWI_Node running in localization_only mode: %d", mode_localization);
    }
    else{
        mode_localization = false;
        ROS_WARN("Localization param not set, running in standard mapping mode");
    }

    int hz;
    if (nodeHandle.getParam(PARAM_MAPRATE, hz)){
        ROS_INFO("Mapping with rate %d", hz);
    }
    else{
        hz = 1; // Default rate = 1
        ROS_WARN("Map Rate param not set, running at standard 1 Hz");
    }

    int tempmode=0;
    if (nodeHandle.getParam(PARAM_MAP_MODE, tempmode)){
        // do nothing
    }
    else{
        ROS_WARN("Map Mode param not set, running in default mode");
    }

    // Create Mapper that will take care of creating transformed, multi-scale maps of environment
    grid_map::GridMap global_map;
    Mapper mapper(nodeHandle, global_map, mapmode(tempmode));
    ros::Timer timer = nodeHandle.createTimer(ros::Duration( 1.0/(double)hz ), std::bind(&Mapper::publishTransformedZoomedMap, &mapper));

    if(mode_localization){
        // Check if there is already a database file present
        // Wait for odometry to be found
        // Then start running

        while(!mapper.receivedMap){
            // Try to get map from database
            // global_map = ...
            // ros::service::waitForService("/rtabmap/")
        }
    }
    

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
};