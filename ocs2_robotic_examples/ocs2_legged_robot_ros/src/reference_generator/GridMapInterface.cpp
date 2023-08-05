#include "ocs2_legged_robot_ros/reference_generator/GridMapInterface.h"

#include <iostream>

namespace ocs2 {
namespace legged_robot {

GridMapInterface::GridMapInterface(::ros::NodeHandle &nh, std::string mapTopic, bool useGridMap)
    : useGridMap_(useGridMap) {
    if (!useGridMap_) {
        return;
    }

    // Define supported layer names
    layers_ = {"elevation", "elevation_smooth", "elevation_roughness"};

    // Map subscriber
    mapSubscriber_ = nh.subscribe(mapTopic, 1, &GridMapInterface::mapCallback, this);

    // Wait for initial map message
    ROS_INFO("Waiting for initial grid map message");
    auto msgPtr = ::ros::topic::waitForMessage<grid_map_msgs::GridMap>(mapTopic);
    if (!msgPtr) {
        std::cerr << "Failed to receive initial grid map message" << std::endl;
        ::ros::shutdown();
        return;
    }
    mapCallback(msgPtr);
}
void GridMapInterface::mapCallback(const grid_map_msgs::GridMap::ConstPtr &msgPtr) {
    grid_map::GridMapRosConverter converter;
    converter.fromMessage(*msgPtr, map_, layers_, false, true);
}

scalar_t GridMapInterface::atPositionElevation(scalar_t x, scalar_t y) {
    // Return 0 if grid map is not used
    if (!useGridMap_) {
        return 0.0;
    }

    pos_ = {x, y};
    return map_.atPosition("elevation", pos_);
}

scalar_t GridMapInterface::atPositionRoughness(scalar_t x, scalar_t y) {
    pos_ = {x, y};
    return map_.atPosition("elevation_roughness", pos_);
}

}  // namespace legged_robot
}  // namespace ocs2