#pragma once

// std-lib
#include <string>
#include <string>

// ros
#include <ros/ros.h>

// ocs2
#include <ocs2_legged_robot/common/Types.h>

// gridmap
#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace ocs2 {
namespace legged_robot {

class GridMapInterface {
   public:
    GridMapInterface(::ros::NodeHandle &nh, std::string mapTopic, bool useGridMap = false);
    scalar_t atPositionElevation(scalar_t x, scalar_t y);
    scalar_t atPositionRoughness(scalar_t x, scalar_t y);
    const grid_map::GridMap &getMap() const { return map_; }

   private:
    void mapCallback(const grid_map_msgs::GridMap::ConstPtr &msgPtr);

    grid_map::GridMap map_;
    grid_map::Position pos_;
    ::ros::Subscriber mapSubscriber_;
    std::vector<std::string> layers_;
    const bool useGridMap_;
};

}  // namespace legged_robot
}  // namespace ocs2