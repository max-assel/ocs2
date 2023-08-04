#include "ocs2_legged_robot_ros/reference_generator/GridMapInterface.h"

#include <iostream>

namespace ocs2 {
namespace legged_robot {

GridMapInterface::GridMapInterface(::ros::NodeHandle &nh, std::string mapTopic, bool useGridMap)
    : useGridMap_(useGridMap) {
    if (!useGridMap_) {
        return;
    }

    // Gridmap layer name
    layer_ = "elevation";

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
    std::vector<std::string> layers = {layer_};
    converter.fromMessage(*msgPtr, map_, layers, false, true);
}

scalar_t GridMapInterface::atPosition(scalar_t x, scalar_t y) {
    if (!useGridMap_) {
        return 0.0;
    }
    pos_[0] = x;
    pos_[1] = y;
    return map_.atPosition(layer_, pos_);
}

}  // namespace legged_robot
}  // namespace ocs2