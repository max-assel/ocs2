#include "ocs2_legged_robot_ros/reference_generator/GridMapInterface.h"

#include <iostream>
#include <chrono>

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

    if (planarTerrainPtr_) {
        return;
    }

    auto convexStart = std::chrono::high_resolution_clock::now();
    // Do convex plane decomposition
    std::unique_ptr<convex_plane_decomposition::PlanarTerrain> newTerrain(
        new convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg)));
    planarTerrainPtr_.swap(newTerrain);
    auto convexEnd = std::chrono::high_resolution_clock::now();
    std::cout << "Convex plane decomposition took "
              << std::chrono::duration_cast<std::chrono::microseconds>(convexEnd - convexStart).count() << " us"
              << std::endl;

    auto sdfStart = std::chrono::high_resolution_clock::now();

    auto &map = getMap();
  
    // Compute signed distance field
    std::vector<std::string> layers{layers_[0]};
    auto &elevationData = map.get(layers[0]);

    // Inpaint if needed.
    if (elevationData.hasNaN()) {
        const float inpaint{elevationData.minCoeffOfFinites()};
        elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v)? v : inpaint; });
    }
    const float heightMargin{0.1};
    const float minValue{elevationData.minCoeffOfFinites()};
    const float maxValue{elevationData.maxCoeffOfFinites() + heightMargin};
    std::unique_ptr<grid_map::SignedDistanceField> newSDF(new grid_map::SignedDistanceField(
        map, layers_[0], minValue, maxValue));
    sdfPtr_.swap(newSDF);

    auto sdfEnd = std::chrono::high_resolution_clock::now();
    std::cout << "Signed distance field computation took "
              << std::chrono::duration_cast<std::chrono::microseconds>(sdfEnd - sdfStart).count() << " us" << std::endl;
    std::cout << "Resolution: " << map.getResolution() << std::endl;
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

}  // namespace legged_robot
}  // namespace ocs2