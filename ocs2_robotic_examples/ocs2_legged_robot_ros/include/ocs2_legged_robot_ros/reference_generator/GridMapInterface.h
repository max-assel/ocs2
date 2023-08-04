#pragma once

// std-lib
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
    GridMapInterface(::ros::NodeHandle &nh, std::string mapTopic);
    scalar_t atPosition(scalar_t x, scalar_t y);

   private:
    void mapCallback(const grid_map_msgs::GridMap::ConstPtr &msgPtr);

    grid_map::GridMap map_;
    grid_map::Position pos_;
    ::ros::Subscriber mapSubscriber_;
    std::string layer_;
};

}  // namespace legged_robot
}  // namespace ocs2