/*
 * MRT_ROS_Anymal.h
 *
 *  Created on: Jun 18, 2018
 *      Author: farbod
 */

#ifndef MRT_ROS_ANYMAL_H_
#define MRT_ROS_ANYMAL_H_

#include <ocs2_quadruped_interface/MRT_ROS_Quadruped.h>

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"

namespace anymal {

class MRT_ROS_Anymal : public switched_model::MRT_ROS_Quadruped<12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<MRT_ROS_Anymal> Ptr;

  typedef switched_model::MRT_ROS_Quadruped<12> BASE;

  typedef OCS2AnymalInterface ocs2_anymal_interface_t;

  /**
   * @param [in] ocs2QuadrupedInterfacePtr: A shared pointer to the quadruped interface class.
   * @param [in] robotName: The name's of the robot.
   */
  MRT_ROS_Anymal(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr, const std::string& pathToConfigFolder)
      : BASE(ocs2QuadrupedInterfacePtr, "anymal") {};

  ~MRT_ROS_Anymal() = default;
};

}  // end of namespace anymal

#endif /* MRT_ROS_ANYMAL_H_ */
