#include "ocs2_legged_robot_ros/reference_generator/GridMapInterface.h"

#include <iostream>

namespace ocs2 {
namespace legged_robot {

GridMapInterface::GridMapInterface(::ros::NodeHandle &nh, std::string mapTopic, bool useGridMap) {
    useGridmap_ = useGridMap;
    if(!useGridMap) {
        return;
    }
    
    // Gridmap layer name
    layers_ = {"elevation", "elevation_smooth", "elevation_roughness"};

    // Map subscriber
    mapSubscriber_ = nh.subscribe(mapTopic, 1, &GridMapInterface::mapCallback, this);

    // Wait for initial map message
    ROS_INFO("Waiting for initial grid map message");
    auto msgPtr = ros::topic::waitForMessage<convex_plane_decomposition_msgs::PlanarTerrain>(mapTopic);
    if (!msgPtr) {
        std::cerr << "Failed to receive initial grid map message" << std::endl;
        ros::shutdown();
        return;
    }
    mapCallback(msgPtr);
}

void GridMapInterface::mapCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr &msg) {

    if(!planarTerrainPtr) {
        std::unique_ptr<convex_plane_decomposition::PlanarTerrain> newTerrain(
            new convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg)));
        planarTerrainPtr.swap(newTerrain);

        auto &map = getMap();
        std::cout << "Grid map layers:" << std::endl;
        for(const auto &layer : map.getLayers()) {
            std::cout << layer << std::endl;
        }
    }
}

scalar_t GridMapInterface::atPositionElevation(scalar_t x, scalar_t y) {
    if(!useGridmap_) {
        return 0.0;
    }



    const auto &map = getMap();
    pos_[0] = x;
    pos_[1] = y;

    if(!map.isInside(pos_)) {
        return 0.0;
    }

    return map.atPosition("elevation", pos_);
}


scalar_t GridMapInterface::atPositionElevationSmooth(scalar_t x, scalar_t y) {
    if(!useGridmap_) {
        return 0.0;
    }



    const auto &map = getMap();
    pos_[0] = x;
    pos_[1] = y;

    if(!map.isInside(pos_)) {
        return 0.0;
    }

    return map.atPosition("smooth_planar", pos_);
}

}  // namespace legged_robot
}  // namespace ocs2