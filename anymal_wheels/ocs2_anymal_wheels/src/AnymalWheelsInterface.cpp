//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

#include <ros/package.h>

#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsCom.h>
#include <ocs2_anymal_wheels_switched_model/core/AnymalWheelsKinematics.h>

namespace anymal {

std::unique_ptr<switched_model::QuadrupedInterface> getAnymalWheelsInterface(const std::string& taskName) {
  std::string taskFolder = getTaskFileFolderWheels(taskName);
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  auto kin = AnymalWheelsKinematics();
  auto kinAd = AnymalWheelsKinematicsAd();
  auto com = AnymalWheelsCom();
  auto comAd = AnymalWheelsComAd();
  return std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedInterface(kin, kinAd, com, comAd, taskFolder));
}

std::string getTaskFileFolderWheels(const std::string& taskName) {
  std::string taskFolder = ros::package::getPath("ocs2_anymal_wheels") + "/config/" + taskName;
  return taskFolder;
}

std::string getTaskFilePathWheels(const std::string& taskName) {
  return getTaskFileFolderWheels(taskName) + "/task.info";
}

}  // namespace anymal
