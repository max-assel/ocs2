#pragma once

#include <string>
#include <unordered_map>
#include <array>
#include <ros/ros.h>
#include <boost/circular_buffer.hpp>

// gridmap
#include <convex_plane_decomposition/PolygonTypes.h>

#include <ocs2_legged_robot_ros/reference_generator/AnymalInverseKinematics.h>
#include <ocs2_legged_robot_ros/reference_generator/GridMapInterface.h>
#include "ocs2_legged_robot/gait/GaitSchedule.h"
#include <ocs2_legged_robot/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>

namespace ocs2 {
namespace legged_robot {

enum class FootState { CONTACT, NEW_CONTACT, SWING, NEW_SWING };
using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
using matrix34_t = Eigen::Matrix<scalar_t, 3, 4>;

class ReferenceGenerator : public SolverSynchronizedModule {
   public:
    // Constructor
    ReferenceGenerator(::ros::NodeHandle &nh, const std::string &referenceFile, const CentroidalModelInfo &modelInfo,
                       SwitchedModelReferenceManager &referenceManager, const PinocchioInterface &pinocchioInterface,
                       const ModelSettings &modelSettings,
                       std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPlannerPtr, std::string gridMapTopic,
                       GaitSchedule &gaitSchedule, bool useGridMap = false);

    // Generate reference trajectory for MPC before each solver run
    void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                      const ReferenceManagerInterface &referenceManager) override;

    // Do nothing after MPC solver run
    void postSolverRun(const PrimalSolution &primalSolution) override{};

   private:
    void computeSamplingTimes(scalar_t initTime, scalar_t finalTime, const ReferenceManagerInterface &referenceManager);
    void computeContactFlags(const ReferenceManagerInterface &referenceManager);
    void computeBaseTrajectory(const vector_t &currentState, const ReferenceManagerInterface &referenceManager);
    void computeFootTrajectories(const vector3_t &currentState, const ReferenceManagerInterface &referenceManager);
    void setContactHeights(scalar_t currentTime);
    void computeFootTrajectoriesZ();
    void computeBaseTrajectoryZ();
    void computeInverseKinematics(const vector_t &currentState);
    void generateReferenceTrajectory();
    inline void setReferenceTrajectory() { referenceManager_.setTargetTrajectories(targetTrajectory_); }


    // helper functions
    constexpr scalar_t getStanceTime();
    FootState getFootState(bool currentContact, bool nextContact);
    vector3_t raibertHeuristic(const vector6_t &basePose, const vector3_t &hipPosition, vector3_t &previousFootPosition, size_t legIdx,
                               scalar_t phase, scalar_t stanceTime);
    void optimizeFoothold(vector3_t &nominalFoothold, vector6_t &nominalBasePose, const scalar_t time,
                          const scalar_t currentPhase, bool firstTouchdown, size_t legIdx, int touchdownIdx,
                          int liftOffIdx);

    void generateFootName2IndexMap();
    size_t footName2Index(const std::string &footName) { return footName2IndexMap_[footName]; }
    size_t footName2Index(const char *footName) { return footName2IndexMap_[std::string(footName)]; }

    void computeFullBasePose(vector6_t &basePose, matrix34_t &hipPositions, scalar_t yawsin, scalar_t yawcos);
    void computeNominalHipPositions(const vector6_t &basePose, matrix34_t &hipPositions, scalar_t yawsin,
                                    scalar_t yawcos);
    inline matrix34_t getDefaultProjectedHipPositions(){return defaultProjectedHipPositions_;};
    void generateDefaultProjectedHipPositions();

   public:
    std::vector<scalar_t> samplingTimes_;
    std::vector<std::array<bool, 4>> contactFlagsAt_;
    std::vector<std::array<bool, 4>> contactFlagsNext_;
    std::vector<vector6_t> baseTrajectory_;
    std::vector<std::array<vector3_t, 4>> footTrajectories_;
    std::array<vector3_t, 4> nextOptimizedFootholds_;
    bool firstRun_;
    std::vector<vector_t> jointTrajectories_;
    feet_array_t<scalar_array_t> liftOffHeightSequence_;
    feet_array_t<scalar_array_t> touchDownHeightSequence_;
    feet_array_t<vector3_t> lastFootholds_;
    TargetTrajectories targetTrajectory_;
    GaitSchedule &gaitSchedule_;
    GridMapInterface gridMapInterface_;

    size_t idxLower_;
    size_t idxUpper_;

   private:
    const scalar_t teps_;
    std::unordered_map<std::string, size_t> footName2IndexMap_;

    ::ros::Subscriber joystickSubscriber_;
    PinocchioInterface pinocchioInterface_;
    SwitchedModelReferenceManager &referenceManager_;
    const CentroidalModelInfo &modelInfo_;
    std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPlannerPtr_;
    ModelSettings modelSettings_;
    AnymalInverseKinematics anymalInverseKinematics_;

    // Commanded velocity and yaw rate
    float vx_;
    float vy_;
    float yrate_;

    // Optimize footholds?
    bool optimizeFootholds_;
    bool useGridmap_;

    // Book-keeping for FootholdPlacementConstraint
    feet_array_t<bool> firstContactInitialized_;
    feet_array_t<scalar_t> firstContactTimes_;
    feet_array_t<Eigen::Isometry3d> convexRegionWorldTransforms_;
    feet_array_t<convex_plane_decomposition::CgalPolygon2d> convexRegions_;

    std::vector<matrix34_t> hipPositions_;
    matrix34_t defaultProjectedHipPositions_;

    // Default base height above the ground
    scalar_t comHeight_;

    // Defalt joint state
    vector_t defaultJointState_;

    // Book keeping
    size_t counter;

    // Statistics
    boost::circular_buffer<scalar_t> genTimes_;
};

}  // namespace legged_robot
}  // namespace ocs2