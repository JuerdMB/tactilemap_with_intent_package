cmake_minimum_required(VERSION 3.0.2)
project(tactilemap_with_intent_package)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    OpenCV
    cv_bridge
    costmap_2d
    roscpp
    std_msgs
    nav_msgs
    grid_map_core
    grid_map_ros
    tf
    tf_conversions
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tactilemap_with_intent_package
  CATKIN_DEPENDS
        std_msgs
        costmap_2d
        roscpp
        tf
        tf_conversions
        grid_map_core
        grid_map_ros
        nav_msgs
        cv_bridge
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
        include
)

# Tactile map main node
add_executable(tmwi_node src/tmwi_node.cpp src/Mapper.cpp)
target_link_libraries(tmwi_node ${catkin_LIBRARIES})

add_executable(test_node src/display_test_node.cpp src/Mapper.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})