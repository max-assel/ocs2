#include <pinocchio/fwd.hpp>

#include "ocs2_legged_robot_ros/reference_generator/ReferenceGenerator.h"

// Joystick message
#include <sensor_msgs/Joy.h>

// Mode schedule
#include <ocs2_core/reference/ModeSchedule.h>

// Contact timings
#include <ocs2_legged_robot/gait/LegLogic.h>

// Display
#include <ocs2_core/misc/Display.h>

// chrono
#include <chrono>

//  Load data
#include <ocs2_core/misc/LoadData.h>

// contact flags
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

// helper functions
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

// Pinocchio
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>

/*

    generateBaseReference(...)
        - generate a reference base trajectory assuming the
          current commanded velocities

    generateFootholdReference(...)
        - generate reference foothold positions given the current elevation
          map and generated reference base trajectory

    generateJointAngleReference(...)
        - generate reference foot positions given reference base trajectory
          and reference foothold positions and compute inverse kinematics
          to get joint angles

*/

namespace ocs2 {
namespace legged_robot {

// TODO: load these constants from the config files
constexpr float VX_MAX_VELOCITY = 0.3;
constexpr float VY_MAX_VELOCITY = 0.2;
constexpr float YAW_RATE_MAX_VELOCITY = 0.5;

ReferenceGenerator::ReferenceGenerator(
    ::ros::NodeHandle &nh, const std::string &referenceFile,
    const CentroidalModelInfo &modelInfo,
    ReferenceManagerInterface &referenceManager,
    const PinocchioInterface &pinocchioInterface)
    : vx_(0.0),
      vy_(0.0),
      yawRate_(0.0),
      defaultJointState_(12),
      modelInfo_(modelInfo),
      referenceManager_(referenceManager),
      counter(0),
      pinocchioInterface_(pinocchioInterface) {
    // Joystick message callback
    auto joy_callback = [this](const sensor_msgs::Joy::ConstPtr &msg) {
        this->vx_ = msg->axes[1] * VX_MAX_VELOCITY;
        this->vy_ = msg->axes[0] * VY_MAX_VELOCITY;
        this->yawRate_ = msg->axes[3] * YAW_RATE_MAX_VELOCITY;
    };

    // Initialize joystick subscriber
    joystickSubscriber_ =
        nh.subscribe<sensor_msgs::Joy>("joy_ramped", 1, joy_callback);

    // Load reference COM height
    loadData::loadCppDataType(referenceFile, "comHeight", comHeight_);

    // Load reference joint state
    loadData::loadEigenMatrix(referenceFile, "defaultJointState",
                              defaultJointState_);
}

void ReferenceGenerator::preSolverRun(
    scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
    const ReferenceManagerInterface &referenceManager) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Planned mode schedule
    ModeSchedule modeSchedule = referenceManager.getModeSchedule();
    scalar_t finalEventTime = modeSchedule.eventTimes.back();
    std::cout << "Final prediction time is: " << finalTime << std::endl;
    std::cout << "Final event time is: " << finalEventTime << std::endl;
    std::cout << "Prediction horizon: " << finalTime - initTime << " s."
              << std::endl;

    feet_array_t<std::vector<bool>> contactFlags;

    // TODO: rewrite this piece of code, it's crap
    constexpr scalar_t time_eps = 1e-5;
    std::vector<scalar_t> interestingTimePoints;
    scalar_t t_i, t_ip, t_mid;
    int N = -1;
    for (size_t i = 0; i < modeSchedule.eventTimes.size() - 1; ++i) {
        t_i = modeSchedule.eventTimes[i];

        if (t_i > initTime + 1.0) {
            break;
        }

        // Only look into the future
        if (t_i <= initTime) {
            continue;
        } else {
            if (N <= 0) {
                size_t diff = modeSchedule.eventTimes.size() - i;
                N = diff * 2 + 2;
                interestingTimePoints.reserve(N);
                interestingTimePoints.push_back(initTime);
            }
        }

        t_ip = modeSchedule.eventTimes[i + 1];
        t_mid = (t_i + t_ip) * 0.5;
        interestingTimePoints.push_back(t_i + time_eps);
        interestingTimePoints.push_back(t_mid);
    }
    interestingTimePoints.push_back(t_ip + time_eps);

#define X_IDX 0
#define Y_IDX 1
#define Z_IDX 2
#define YAW_IDX 3
#define PITCH_IDX 4
#define ROLL_IDX 5

    // Generate pose reference
    const vector_t currentPose = currentState.segment<6>(6);
    vector_array_t poseReference;
    poseReference.push_back(currentPose);

    for (size_t i = 1; i < interestingTimePoints.size(); ++i) {
        scalar_t dt = interestingTimePoints[i] - interestingTimePoints[i - 1];
        scalar_t last_yaw = poseReference[i - 1][YAW_IDX];

        // transform from body frame to world frame
        scalar_t c = std::cos(last_yaw);
        scalar_t s = std::sin(last_yaw);
        scalar_t v_x_world = vx_ * c - vy_ * s;
        scalar_t v_y_world = vx_ * s + vy_ * c;

        // Integrate
        scalar_t new_x = poseReference[i - 1][X_IDX] + v_x_world * dt;
        scalar_t new_y = poseReference[i - 1][Y_IDX] + v_y_world * dt;
        scalar_t new_yaw = last_yaw + yawRate_ * dt;

        vector_t new_pose =
            (vector_t(6) << new_x, new_y, comHeight_, new_yaw, 0.0, 0.0)
                .finished();
        poseReference.push_back(new_pose);
    }

    // const scalar_array_t timeTrajectory = interestingTimePoints;
    scalar_array_t timeTrajectory = interestingTimePoints;
    vector_array_t stateTrajectory(timeTrajectory.size(),
                                   vector_t::Zero(currentState.size()));
    for (size_t i = 0; i < timeTrajectory.size(); ++i) {
        stateTrajectory[i] << vector_t::Zero(6), poseReference[i],
            defaultJointState_;
    }
    const vector_array_t inputTrajectory(timeTrajectory.size(),
                                         vector_t::Zero(modelInfo_.inputDim));

    const TargetTrajectories targetTrajectories(timeTrajectory, stateTrajectory,
                                                inputTrajectory);

    referenceManager_.setTargetTrajectories(targetTrajectories);

    // Generate foot positions for interesting time points [x,y,z] expressed in
    // world frame
    std::vector<std::vector<vector3_t>> footPositions;
    footPositions.resize(4);
    for (size_t i = 0; i < footPositions.size(); ++i) {
        footPositions[i].resize(interestingTimePoints.size());
    }

    feet_array_t<std::vector<bool>> contactFlagsAtInterestingTimePoints;
    std::for_each(
        contactFlagsAtInterestingTimePoints.begin(),
        contactFlagsAtInterestingTimePoints.end(),
        [&](std::vector<bool> &v) { v.reserve(interestingTimePoints.size()); });
    for (size_t i = 0; i < interestingTimePoints.size(); ++i) {
        scalar_t time = interestingTimePoints[i];
        size_t modeIdx = modeSchedule.modeAtTime(time);
        contact_flag_t contactFlag = modeNumber2StanceLeg(modeIdx);
        for (size_t j = 0; j < 4; ++j) {
            contactFlagsAtInterestingTimePoints[j].push_back(contactFlag[j]);
        }
    }
    // update pinocchio state
    auto generalizedState =
        centroidal_model::getGeneralizedCoordinates(currentState, modelInfo_);
    const PinocchioInterface::Model &model = pinocchioInterface_.getModel();
    PinocchioInterface::Data &data = pinocchioInterface_.getData();

    // Frame name of interest
    const std::string foot_name = "RF_FOOT";

    std::cout << "frame ids:" << std::endl;
    std::cout << "RF FOOT: " << model.getFrameId("RF_FOOT") << std::endl;
    std::cout << "FRdddd FOOT: " << model.getFrameId("FRdddd_FOOT")
              << std::endl;
    std::cout << model.nframes << std::endl;

    // compute forward kinematics
    std::cout << model.nq << "\t" << generalizedState.size() << std::endl;
    std::cout << generalizedState.transpose() << std::endl;
    ::pinocchio::forwardKinematics(model, data, generalizedState);

    // Update frame placement for 'foot_name'
    ::pinocchio::updateFramePlacements(model, data);

    // Get foot position
    auto world_position = data.oMf[model.getFrameId(foot_name)].translation();

    auto end_time = std::chrono::high_resolution_clock::now();

    std::cout << "FR FOOT position: " << world_position.transpose()
              << std::endl;baseTrajectory_
    std::cout << "mass: " << data.mass[1] << std::endl;
    std::cout << "mass: " << data.mass.size() << std::endl;
    // std::cout << "Generalized state" << generalizedState.transpose() <<
    // std::endl;
    if (counter == 33) {
        counter = 0;
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time);
        std::cout << "Reference generation took " << duration.count() << " us"
                  << std::endl;
        std::cout << timeTrajectory[timeTrajectory.size() - 1] -
                         timeTrajectory[0]
                  << std::endl;
        std::cout << "{" << toDelimitedString(interestingTimePoints) << "}"
                  << std::endl;
        std::cout << "{"
                  << toDelimitedString(contactFlagsAtInterestingTimePoints[0])
                  << "}" << std::endl;
        std::cout << "{"
                  << toDelimitedString(contactFlagsAtInterestingTimePoints[1])
                  << "}" << std::endl;
        std::cout << "{"
                  << toDelimitedString(contactFlagsAtInterestingTimePoints[2])
                  << "}" << std::endl;
        std::cout << "{"
                  << toDelimitedString(contactFlagsAtInterestingTimePoints[3])
                  << "}" << std::endl;
        std::cout << modeSchedule.modeAtTime(interestingTimePoints[3])
                  << std::endl;
    } else {
        ++counter;
    }

    // // Contact and swing timings
    // auto contactTimings = extractContactTimingsPerLeg(modeSchedule);
    // auto swingTimings = extractSwingTimingsPerLeg(modeSchedule);
    // for (size_t i = 0; i < contactTimings[0].size(); ++i) {
    //     std::cout << "Contact timings: ";
    //     std::cout << contactTimings[0][2].start << " ";
    //     std::cout << contactTimings[0][2].end << " \t";
    //     std::cout << "Swing timings: ";
    //     std::cout << swingTimings[0][2].start << " ";
    //     std::cout << swingTimings[0][2].end << " \t";
    //     break;
    // }
    // std::cout << std::endl;

    for (size_t i = 1; i < interestingTimePoints.size(); ++i) {
        scalar_t time_curr = interestingTimePoints[i];
        bool contact_prev = contactFlagsAtInterestingTimePoints[0][i - 1];
        bool contact_curr = contactFlagsAtInterestingTimePoints[0][i];
        bool new_contact = !contact_prev && contact_curr;

        if (new_contact) {
            // get contact phases
            auto contactPhases = getContactPhasePerLeg(time_curr, modeSchedule);
            scalar_t t_mid = interestingTimePoints[i + 1];
            auto contactPhasesMid = getContactPhasePerLeg(t_mid, modeSchedule);
            std::cout << contactPhases[0].phase << std::endl;
            std::cout << contactPhasesMid[0].phase << std::endl;
            break;
        }
    }
}

}  // namespace legged_robot
}  // namespace ocs2