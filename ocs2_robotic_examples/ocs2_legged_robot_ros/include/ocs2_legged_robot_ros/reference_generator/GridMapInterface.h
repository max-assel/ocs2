#pragma once

// std-lib
#include <memory>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>

// gridmap
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

namespace ocs2 {
namespace legged_robot {

using scalar_t = double;

class GridMapInterface {
   public:
    GridMapInterface(::ros::NodeHandle &nh, std::string mapTopic, bool useGridMap);
    scalar_t atPositionElevation(scalar_t x, scalar_t y);
    inline grid_map::GridMap &getMap() { return planarTerrainPtr_->gridMap; }
    inline grid_map::SignedDistanceField &getSDF() { return *sdfPtr_; }
    inline std::vector<convex_plane_decomposition::PlanarRegion> &getPlanarRegions() {
        return planarTerrainPtr_->planarRegions;
    }

   private:
    void mapCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr &msgPtr);

    grid_map::Position pos_;
    ::ros::Subscriber mapSubscriber_;
    std::vector<std::string> layers_;

    std::unique_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
    std::unique_ptr<grid_map::SignedDistanceField> sdfPtr_;

    bool useGridmap_;
};

}  // namespace legged_robot
}  // namespace ocs2