/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>

#include <ocs2_mpc/MPC_Settings.h>
#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))

  // Initialize ros node
  ros::init(argc, argv, "anymal_wheels_mrt");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalWheelsInterface(taskName);
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(anymal::getTaskFilePathWheels(taskName));
  quadrupedDummyNode(nodeHandle, *anymalInterface, &anymalInterface->getRollout(), mpcSettings.mrtDesiredFrequency_,
                     mpcSettings.mpcDesiredFrequency_);

  return 0;
}
