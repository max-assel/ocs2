#include <pinocchio/fwd.hpp>

#include "ocs2_legged_robot_ros/reference_generator/ReferenceGenerator.h"

// std-lib
#include <chrono>
#include <iomanip>

// Pinocchio
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/math/rpy.hpp>

// Ros
#include <sensor_msgs/Joy.h>

// gridmap
#include <convex_plane_decomposition/ConvexRegionGrowing.h>

// Mode schedule
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_legged_robot/gait/LegLogic.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Display.h>

namespace ocs2 {
namespace legged_robot {

constexpr size_t X_IDX = 0;
constexpr size_t Y_IDX = 1;
constexpr size_t Z_IDX = 2;
constexpr size_t YAW_IDX = 3;
constexpr size_t PITCH_IDX = 4;
constexpr size_t ROLL_IDX = 5;
constexpr char NEWLINE = '\n';

// TODO: load these constants from the config files
constexpr float VX_MAX_VELOCITY = 0.9;
constexpr float VY_MAX_VELOCITY = 0.2;
constexpr float YAW_RATE_MAX_VELOCITY = 1.0;

constexpr scalar_t BUFFER_SIZE = 100;

ReferenceGenerator::ReferenceGenerator(::ros::NodeHandle &nh, const std::string &referenceFile,
                                       const CentroidalModelInfo &modelInfo,
                                       SwitchedModelReferenceManager &referenceManager,
                                       const PinocchioInterface &pinocchioInterface, const ModelSettings &modelSettings,
                                       std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPlannerPtr,
                                       std::string gridMapTopic, GaitSchedule &gaitSchedule, bool useGridMap)
    : vx_(0.0),
      vy_(0.0),
      yrate_(0.0),
      defaultJointState_(12),
      modelInfo_(modelInfo),
      referenceManager_(referenceManager),
      pinocchioInterface_(pinocchioInterface),
      swingTrajectoryPlannerPtr_(swingTrajectoryPlannerPtr),
      modelSettings_(modelSettings),
      anymalInverseKinematics_(pinocchioInterface_, modelSettings_, referenceFile),
      gridMapInterface_(nh, gridMapTopic, useGridMap),
      gaitSchedule_(gaitSchedule),
      counter(0),
      teps_(1e-6),
      genTimes_(BUFFER_SIZE),
      useGridmap_(useGridMap),
      firstRun_(true) {
    // Setup joystick subscriber
    auto joy_callback = [this](const sensor_msgs::Joy::ConstPtr &msg) {
        this->vx_ = msg->axes[1] * VX_MAX_VELOCITY;
        this->vy_ = msg->axes[0] * VY_MAX_VELOCITY;
        this->yrate_ = msg->axes[3] * YAW_RATE_MAX_VELOCITY;
    };
    joystickSubscriber_ = nh.subscribe<sensor_msgs::Joy>("joy_ramped", 1, joy_callback);

    // Load reference COM height
    loadData::loadCppDataType(referenceFile, "comHeight", comHeight_);

    // Load reference joint state
    loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState_);

    // Should we optimize footholds?
    loadData::loadCppDataType(referenceFile, "optimizeFootholds", optimizeFootholds_);

    // Generate foot name to index map
    generateFootName2IndexMap();

    // Compute default projected hip positions
    generateDefaultProjectedHipPositions();

    // Initialize book-kepping variables
    firstContactInitialized_ = {false, false, false, false};

    // Initialize reference manager pointers
    std::cout << "Initializing reference manager pointers" << std::endl;
    referenceManager_.firstContactInitializedPtr = &firstContactInitialized_;
    referenceManager_.firstContactTimesPtr = &firstContactTimes_;
    referenceManager_.convexRegionWorldTransformsPtr = &convexRegionWorldTransforms_;
    referenceManager_.convexRegionsPtr = &convexRegions_;
}

void ReferenceGenerator::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                                      const ReferenceManagerInterface &referenceManager) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Compute sampling times
    auto start_t1 = std::chrono::high_resolution_clock::now();
    computeSamplingTimes(initTime, finalTime, referenceManager);
    auto end_t1 = std::chrono::high_resolution_clock::now();
    auto duration_t1 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t1 - start_t1).count() / 1e3;

    // Compute contact flags
    auto start_t2 = std::chrono::high_resolution_clock::now();
    computeContactFlags(referenceManager);
    auto end_t2 = std::chrono::high_resolution_clock::now();
    auto duration_t2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t2 - start_t2).count() / 1e3;

    // Compute base trajectory
    auto start_t3 = std::chrono::high_resolution_clock::now();
    computeBaseTrajectory(currentState, referenceManager);
    auto end_t3 = std::chrono::high_resolution_clock::now();
    auto duration_t3 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t3 - start_t3).count() / 1e3;

    // Compute foot trajectories ... we are bottlenecking here
    auto start_t4 = std::chrono::high_resolution_clock::now();
    computeFootTrajectories(currentState, referenceManager);
    auto end_t4 = std::chrono::high_resolution_clock::now();
    auto duration_t4 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t4 - start_t4).count() / 1e3;

    auto start_t5 = std::chrono::high_resolution_clock::now();
    setContactHeights(initTime);
    auto end_t5 = std::chrono::high_resolution_clock::now();
    auto duration_t5 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t5 - start_t5).count() / 1e3;

    auto start_t6 = std::chrono::high_resolution_clock::now();
    computeFootTrajectoriesZ();
    auto end_t6 = std::chrono::high_resolution_clock::now();
    auto duration_t6 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t6 - start_t6).count() / 1e3;

    auto start_t7 = std::chrono::high_resolution_clock::now();
    computeBaseTrajectoryZ();
    auto end_t7 = std::chrono::high_resolution_clock::now();
    auto duration_t7 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t7 - start_t7).count() / 1e3;

    // Compute inverse kinematics
    auto start_t8 = std::chrono::high_resolution_clock::now();
    computeInverseKinematics(currentState);
    auto end_t8 = std::chrono::high_resolution_clock::now();
    auto duration_t8 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t8 - start_t8).count() / 1e3;

    // Generate reference trajectory
    auto start_t9 = std::chrono::high_resolution_clock::now();
    generateReferenceTrajectory();
    auto end_t9 = std::chrono::high_resolution_clock::now();
    auto duration_t9 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t9 - start_t9).count() / 1e3;

    // Set reference trajectory
    auto start_t10 = std::chrono::high_resolution_clock::now();
    setReferenceTrajectory();
    auto end_t10 = std::chrono::high_resolution_clock::now();
    auto duration_t10 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t10 - start_t10).count() / 1e3;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1e3;

    genTimes_.push_back(duration);

    // Print statistics
    constexpr size_t STATISTICS_PRINT_INTERVAL = 33;
    if (counter == STATISTICS_PRINT_INTERVAL) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Reference generation took " << duration << " us" << NEWLINE;
        std::cout << "Computing sampling times took " << duration_t1 << " us" << NEWLINE;
        std::cout << "Computing contact flags took " << duration_t2 << " us" << NEWLINE;
        std::cout << "Computing base trajectory took " << duration_t3 << " us" << NEWLINE;
        std::cout << "Computing foot trajectories took " << duration_t4 << " us" << NEWLINE;
        std::cout << "Setting contact heights took " << duration_t5 << " us" << NEWLINE;
        std::cout << "Computing foot trajectories Z took " << duration_t6 << " us" << NEWLINE;
        std::cout << "Computing base trajectory Z took " << duration_t7 << " us" << NEWLINE;
        std::cout << "Computing inverse kinematics took " << duration_t8 << " us" << NEWLINE;
        std::cout << "Generating reference trajectory took " << duration_t9 << " us" << NEWLINE;
        std::cout << "Setting reference trajectory took " << duration_t10 << " us" << NEWLINE;
        std::cout << "Average generation time: "
                  << std::accumulate(genTimes_.begin(), genTimes_.end(), 0.0) / genTimes_.size() << " us" << NEWLINE;
        std::cout << std::endl;
        counter = 0;
    } else {
        ++counter;
    }

    // Set latest signed distance field pointer
    referenceManager_.sdfPtr = &gridMapInterface_.getSDF();

    firstRun_ = false;
}

void ReferenceGenerator::computeSamplingTimes(scalar_t initTime, scalar_t finalTime,
                                              const ReferenceManagerInterface &referenceManager) {
    // 1. generate mode schedule
    const auto &modeSchedule = referenceManager.getModeSchedule();

    // 2. unpack event times
    const auto &eventTimes = modeSchedule.eventTimes;  // eventTimes = std::vector<scalar_t>

    // 3. get first index in eventTimes vector that's larger or equal to than initTime
    // std::lower_bound finds such a value x that is >= the input value
    auto idxLower_iter = std::lower_bound(eventTimes.begin(), eventTimes.end(), initTime);
    idxLower_ = std::distance(eventTimes.begin(), idxLower_iter);

    // 4. get first index in eventTimes vector such that finalTime is strictly smaller than the value x
    auto idxUpper_iter = std::upper_bound(idxLower_iter, eventTimes.end(), finalTime);
    idxUpper_ = std::distance(idxLower_iter, idxUpper_iter) + idxLower_;

    // 5. the first sampling time is the current time
    samplingTimes_.clear();
    size_t N = idxUpper_ - idxLower_ + 1;
    bool initTimeIsEventTime = eventTimes[idxLower_] == initTime;
    N = initTimeIsEventTime ? N : N + 1;
    samplingTimes_.reserve(N);
    if (!initTimeIsEventTime) {
        samplingTimes_.push_back(initTime);
    }

    // 6. generate the rest of the sampling times
    samplingTimes_.insert(samplingTimes_.end(), idxLower_iter, idxUpper_iter + 1);
}

void ReferenceGenerator::computeContactFlags(const ReferenceManagerInterface &referenceManager) {
    // 1. generate mode schedule
    const auto &modeSchedule = referenceManager.getModeSchedule();

    auto computeContactFlags = [this, &modeSchedule](std::vector<std::array<bool, 4>> &flags, scalar_t eps) {
        flags.clear();
        flags.reserve(samplingTimes_.size());
        size_t modeIdx;
        for (size_t i = 0; i < samplingTimes_.size(); ++i) {
            modeIdx = modeSchedule.modeAtTime(samplingTimes_[i] + eps);
            flags.push_back(modeNumber2StanceLeg(modeIdx));
        }
    };

    // 2. generate contact flags
    computeContactFlags(contactFlagsAt_, 0.0);
    computeContactFlags(contactFlagsNext_, teps_);
}

void ReferenceGenerator::computeBaseTrajectory(const vector_t &currentState,
                                               const ReferenceManagerInterface &referenceManager) {
    // 1. Set base trajectory size
    baseTrajectory_.resize(samplingTimes_.size());
    hipPositions_.resize(samplingTimes_.size());

    // 2. Get current base pose
    vector6_t basePose = centroidal_model::getBasePose(currentState, modelInfo_);

    // 3. Compute nominal base pose for initial time
    scalar_t yawsin = std::sin(basePose[YAW_IDX]);
    scalar_t yawcos = std::cos(basePose[YAW_IDX]);
    computeFullBasePose(basePose, hipPositions_[0], yawsin, yawcos);
    baseTrajectory_[0] = basePose;

    // 4. Compute nominal base pose for the rest of the trajectory
    scalar_t dt, vx_world, vy_world;
    for (size_t i = 1; i < samplingTimes_.size(); ++i) {
        // Compute sine and cosine of the current yaw angle
        yawsin = std::sin(basePose[YAW_IDX]);
        yawcos = std::cos(basePose[YAW_IDX]);

        // Express velocities in the world frame
        vx_world = vx_ * yawcos - vy_ * yawsin;
        vy_world = vx_ * yawsin + vy_ * yawcos;

        // Compute time step
        dt = samplingTimes_[i] - samplingTimes_[i - 1];

        // Integrate
        basePose[X_IDX] += vx_world * dt;
        basePose[Y_IDX] += vy_world * dt;
        basePose[YAW_IDX] += yrate_ * dt;

        // Compute base orientation and store hip positions
        computeFullBasePose(basePose, hipPositions_[i], yawsin, yawcos);

        baseTrajectory_[i] = basePose;
    }
}

void ReferenceGenerator::computeFullBasePose(vector6_t &basePose, matrix34_t &hipPositions, scalar_t yawsin,
                                             scalar_t yawcos) {
    // Nominal hip positions considering no roll and pitch
    computeNominalHipPositions(basePose, hipPositions, yawsin, yawcos);

    // Compute nominal hip position Z coordinate
    for (size_t i = 0; i < 4; ++i) {
        hipPositions(2, i) =
            gridMapInterface_.atPositionElevationSmooth(hipPositions(X_IDX, i), hipPositions(Y_IDX, i)) + comHeight_;
    }

    // Fit a plane through the nominal hip positions
    matrix_t A(4, 3);
    A.block<4, 2>(0, 0) = hipPositions.transpose().block<4, 2>(0, 0);
    A.block<4, 1>(0, 2) = vector_t::Ones(4);
    vector_t b = -hipPositions.transpose().col(2);

    const vector_t planeParams = A.colPivHouseholderQr().solve(b);
    const scalar_t an = planeParams[0];
    const scalar_t bn = planeParams[1];
    const scalar_t cn = 1.0;
    const scalar_t dn = planeParams[2];

    // plane normal vector
    vector_t n = (vector_t(3) << an, bn, cn).finished();
    n = n / n.norm();

    // Nominal direction of the x-axis
    vector_t x_axis = (vector_t(3) << yawcos, yawsin, 0.0).finished();

    // Nominal direction of the y-axis
    vector_t y_axis = (vector_t(3) << -yawsin, yawcos, 0.0).finished();

    // Projected x-axis
    vector_t x_proj = x_axis - x_axis.dot(n) * n;
    x_proj = x_proj / x_proj.norm();

    // Projected y-axis
    vector_t y_proj = y_axis - y_axis.dot(n) * n;
    y_proj = y_proj / y_proj.norm();

    // Compute rpy angles
    matrix3_t R;
    R.col(0) = x_proj;
    R.col(1) = y_proj;
    R.col(2) = n;
    const vector_t rpy = pinocchio::rpy::matrixToRpy(R);

    // basePose[YAW_IDX] = rpy[2]  <-- not needed here
    basePose[PITCH_IDX] = rpy[1];
    basePose[ROLL_IDX] = rpy[0];
    basePose[Z_IDX] = hipPositions.row(2).mean();

    // Update hip positions
    hipPositions = R * getDefaultProjectedHipPositions() + basePose.head<3>().replicate<1, 4>();
}

void ReferenceGenerator::computeNominalHipPositions(const vector6_t &basePose, matrix34_t &hipPositions,
                                                    scalar_t yawsin, scalar_t yawcos) {
    matrix3_t R = (matrix3_t() << yawcos, -yawsin, 0.0,  // clang-format off
                                  yawsin,  yawcos, 0.0,
                                  0.0,     0.0,    1.0).finished();  // clang-format on
    hipPositions = R * getDefaultProjectedHipPositions() + basePose.head<3>().replicate<1, 4>();
}

void ReferenceGenerator::generateDefaultProjectedHipPositions() {
    // 1. Create state vector - 6 base states + 12 joint states
    vector_t robotState = (vector_t(6 + 12) << vector_t::Zero(6), defaultJointState_).finished();

    // 2. Unpack pinocchio model and data
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();

    // 3. Compute forward kinematics
    pinocchio::framesForwardKinematics(model, data, robotState);

    // 4. Compute default projected hip positions
    for (size_t i = 0; i < modelSettings_.contactNames3DoF.size(); ++i) {
        size_t frameIdx = model.getFrameId(modelSettings_.contactNames3DoF[i]);
        vector3_t hipPosition = data.oMf[frameIdx].translation();
        hipPosition[2] = 0.0;  // Project onto the ground
        defaultProjectedHipPositions_.col(i) = hipPosition;
    }
}

void ReferenceGenerator::computeFootTrajectories(const vector_t &currentState,
                                                 const ReferenceManagerInterface &referenceManager) {
    // 1. Set feet trajectories size
    footTrajectories_.resize(samplingTimes_.size());

    // 2. Get mode schedule
    const auto &modeSchedule = referenceManager.getModeSchedule();

    // Set last footholds
    for (size_t legIdx = 0; legIdx < modelInfo_.numThreeDofContacts; ++legIdx) {
        FootState state = getFootState(contactFlagsAt_[0][legIdx], contactFlagsNext_[0][legIdx]);
        switch (state) {
            case FootState::CONTACT:
            case FootState::NEW_SWING:
            case FootState::NEW_CONTACT:
                lastFootholds_[legIdx] = footTrajectories_[0][legIdx];
                break;
        }
    }

    // 3. Compute forward kinematics
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(currentState, modelInfo_));
    for (const std::string &footName : modelSettings_.contactNames3DoF) {
        pinocchio::updateFramePlacement(model, data, model.getFrameId(footName));
    }

    // 4. Set initial foot position
    auto setInitialFootPosition = [this, &model, &data](const std::string &footName) {
        footTrajectories_[0][footName2Index(footName)] = data.oMf[model.getFrameId(footName)].translation();
    };
    for (const std::string &footName : modelSettings_.contactNames3DoF) {
        setInitialFootPosition(footName);
    }

    std::array<bool, 4> firstTouchdown = {true, true, true, true};
    auto currentswingPhases = getSwingPhasePerLeg(samplingTimes_[0], modeSchedule);

    for (size_t i = 1; i < samplingTimes_.size(); ++i) {
        // Get current swing phases
        auto swingPhases = getSwingPhasePerLeg(samplingTimes_[i], modeSchedule);
        for (size_t legIdx = 0; legIdx < modelInfo_.numThreeDofContacts; ++legIdx) {
            FootState state = getFootState(contactFlagsAt_[i][legIdx], contactFlagsNext_[i][legIdx]);
            switch (state) {
                case FootState::CONTACT:  // deliberate fall-through
                case FootState::NEW_SWING:
                    footTrajectories_[i][legIdx] = footTrajectories_[i - 1][legIdx];  // copy the last position
                    break;
                case FootState::NEW_CONTACT: {
                    scalar_t stanceTime = getStanceTime();
                    const scalar_t phase = 1.0;
                    footTrajectories_[i][legIdx] =
                        raibertHeuristic(baseTrajectory_[i], hipPositions_[i].col(legIdx),
                                         footTrajectories_[i - 1][legIdx], legIdx, phase, stanceTime);
                    footTrajectories_[i][legIdx][Z_IDX] = baseTrajectory_[i][Z_IDX] - comHeight_;

                    // find next liftoff
                    int liftoff_idx = i+1;
                    while(liftoff_idx < samplingTimes_.size() && contactFlagsNext_[liftoff_idx][legIdx]) {
                        ++liftoff_idx;
                    }
                    

                    optimizeFoothold(footTrajectories_[i][legIdx], baseTrajectory_[i], samplingTimes_[i],
                                     currentswingPhases[legIdx].phase, firstTouchdown[legIdx], legIdx, i,
                                     liftoff_idx);  // Only valid for trot gait
                    firstTouchdown[legIdx] = false;
                } break;
                case FootState::SWING: {
                    scalar_t stanceTime = getStanceTime();
                    const scalar_t phase = swingPhases[legIdx].phase;
                    footTrajectories_[i][legIdx] =
                        raibertHeuristic(baseTrajectory_[i], hipPositions_[i].col(legIdx),
                                         footTrajectories_[i - 1][legIdx], legIdx, phase, stanceTime);
                } break;  // raibert heuristic -> phase = whatever ocs2 tells us
            }
        }
    }
}

void ReferenceGenerator::computeBaseTrajectoryZ() {
    scalar_t total, n_contacts;
    for (size_t i = 1; i < baseTrajectory_.size(); ++i) {
        // Compute average foot height
        n_contacts = 0.0;
        total = 0.0;
        scalar_t lowest = std::numeric_limits<scalar_t>::max();
        scalar_t highest = std::numeric_limits<scalar_t>::min();
        for (size_t j = 0; j < 4; ++j) {
            if (contactFlagsNext_[i][j]) {
                total += footTrajectories_[i][j][Z_IDX];
                if (footTrajectories_[i][j][Z_IDX] < lowest) lowest = footTrajectories_[i][j][Z_IDX];
                if (footTrajectories_[i][j][Z_IDX] > highest) highest = footTrajectories_[i][j][Z_IDX];
                n_contacts += 1.0;
            }
        }

        // Set base height
        if (n_contacts > 0.0) {
            // baseTrajectory_[i][Z_IDX] = lowest + comHeight_;  // some legs in contact
            // baseTrajectory_[i][Z_IDX] = total / n_contacts + comHeight_;  // some legs in contact
            baseTrajectory_[i][Z_IDX] = (lowest + highest) / 2 + comHeight_;  // some legs in contact
        } else if (i > 0) {
            baseTrajectory_[i][Z_IDX] = baseTrajectory_[i - 1][Z_IDX];  // all legs in the air, use previous height
        }
    }
}

void ReferenceGenerator::setContactHeights(scalar_t currentTime) {
    // 1. Get mode sequence and event times
    const auto &modeSchedule = referenceManager_.getModeSchedule();
    const auto &eventTimes = modeSchedule.eventTimes;

    const auto &modeSequence = modeSchedule.modeSequence;
    const auto eesContactFlagStocks = extractContactFlags(modeSequence);

    // 2. resize lift-off and touchdown sequences
    auto resizeArray = [](feet_array_t<scalar_array_t> &arr, size_t size) {
        for (size_t i = 0; i < arr.size(); ++i) {
            arr[i].resize(size);
        }
    };
    resizeArray(liftOffHeightSequence_, eventTimes.size());
    resizeArray(touchDownHeightSequence_, eventTimes.size());

    int offset = eventTimes[idxLower_] != currentTime ? 1 : 0;

    static std::array<scalar_t, 4> lastLiftOffHeight = {0.0, 0.0, 0.0, 0.0};

    // 3. compute lift-off and touchdown sequences
    scalar_t x1, y1, x2, y2;
    for (size_t ii = idxLower_; ii <= idxUpper_ && useGridmap_; ++ii) {
        int i_st = static_cast<int>(ii) - static_cast<int>(idxLower_) + offset;
        for (size_t legIdx = 0; legIdx < modelInfo_.numThreeDofContacts; ++legIdx) {
            bool cont = eesContactFlagStocks[legIdx][ii];
            if (!cont) {  // touchdown
                int start, stop;
                std::tie(start, stop) = swingTrajectoryPlannerPtr_->findIndex(ii, eesContactFlagStocks[legIdx]);
                start = offset + static_cast<int>(start) - static_cast<int>(idxLower_);
                stop = offset + static_cast<int>(stop) - static_cast<int>(idxLower_);
                if (start < offset) {
                    liftOffHeightSequence_[legIdx][ii] = lastLiftOffHeight[legIdx];
                } else {
                    x1 = footTrajectories_[start][legIdx][X_IDX];
                    y1 = footTrajectories_[start][legIdx][Y_IDX];
                    liftOffHeightSequence_[legIdx][ii] = gridMapInterface_.atPositionElevation(x1, y1);
                }

                if (stop >= footTrajectories_.size()) {
                    touchDownHeightSequence_[legIdx][ii] = liftOffHeightSequence_[legIdx][ii];
                } else {
                    x2 = footTrajectories_[stop][legIdx][X_IDX];
                    y2 = footTrajectories_[stop][legIdx][Y_IDX];
                    touchDownHeightSequence_[legIdx][ii] = gridMapInterface_.atPositionElevation(x2, y2);
                }
            } else {
                x1 = footTrajectories_[i_st][legIdx][X_IDX];
                y1 = footTrajectories_[i_st][legIdx][Y_IDX];
                liftOffHeightSequence_[legIdx][ii] = gridMapInterface_.atPositionElevation(x1, y1);
                if (ii == idxLower_) {
                    lastLiftOffHeight[legIdx] = lastFootholds_[legIdx][Z_IDX];
                }
            }
        }
    }

    if(useGridmap_) {
        for (int i = 0; i < 4; ++i) {
            std::fill(liftOffHeightSequence_[i].begin(), liftOffHeightSequence_[i].begin() + idxLower_, lastFootholds_[i][Z_IDX]);
            std::fill(touchDownHeightSequence_[i].begin(), touchDownHeightSequence_[i].begin() + idxLower_, lastFootholds_[i][Z_IDX]);
        }
    }


    if (!useGridmap_) {
        for (int i = 0; i < 4; ++i) {
            std::fill(liftOffHeightSequence_[i].begin(), liftOffHeightSequence_[i].end(), lastFootholds_[i][Z_IDX]);
            std::fill(touchDownHeightSequence_[i].begin(), touchDownHeightSequence_[i].end(), lastFootholds_[i][Z_IDX]);
        }
    }

    // 4. Update swing trajectory planner
    swingTrajectoryPlannerPtr_->update(modeSchedule, liftOffHeightSequence_, touchDownHeightSequence_);
}

void ReferenceGenerator::computeFootTrajectoriesZ() {
    for (size_t i = 1; i < samplingTimes_.size(); ++i) {
        for (size_t legIdx = 0; legIdx < modelInfo_.numThreeDofContacts; ++legIdx) {
            footTrajectories_[i][legIdx][Z_IDX] =
                swingTrajectoryPlannerPtr_->getZpositionConstraint(legIdx, samplingTimes_[i]);
        }
    }
}

void ReferenceGenerator::computeInverseKinematics(const vector_t &currentState) {
    jointTrajectories_.clear();
    jointTrajectories_.reserve(samplingTimes_.size());
    for (size_t i = 0; i < samplingTimes_.size(); ++i) {
        const auto &basePose = baseTrajectory_[i];
        auto jointAngles = anymalInverseKinematics_.anymalInverseKinematics(basePose.head<3>(), basePose.tail<3>(),
                                                                            footTrajectories_[i]);
        jointTrajectories_.push_back(jointAngles);
    }
}
void ReferenceGenerator::generateReferenceTrajectory() {
    vector_array_t stateTrajectory(samplingTimes_.size(), vector_t::Zero(modelInfo_.stateDim));

    for (size_t i = 0; i < samplingTimes_.size(); ++i) {
        // get world velocities
        const scalar_t yaw_s = std::sin(baseTrajectory_[i][YAW_IDX]);
        const scalar_t yaw_c = std::cos(baseTrajectory_[i][YAW_IDX]);

        // express velocity in the world frame
        const scalar_t vx = vx_ * yaw_c - vy_ * yaw_s;
        const scalar_t vy = vx_ * yaw_s + vy_ * yaw_c;

        stateTrajectory[i] << vx, vy, vector_t::Zero(4), baseTrajectory_[i], jointTrajectories_[i];
    }
    const vector_array_t inputTrajectory(samplingTimes_.size(), vector_t::Zero(modelInfo_.inputDim));
    targetTrajectory_.setTimeTrajectory(std::move(samplingTimes_));
    targetTrajectory_.setStateTrajectory(std::move(stateTrajectory));
    targetTrajectory_.setInputTrajectory(std::move(inputTrajectory));
}

constexpr scalar_t ReferenceGenerator::getStanceTime() { return 0.45; }

FootState ReferenceGenerator::getFootState(bool currentContact, bool nextContact) {
    if (currentContact) {
        return nextContact ? FootState::CONTACT : FootState::NEW_SWING;
    }
    return nextContact ? FootState::NEW_CONTACT : FootState::SWING;
}

vector3_t ReferenceGenerator::raibertHeuristic(const vector6_t &basePose, const vector3_t &hipPosition,
                                               vector3_t &footPositionPrev, size_t legIdx, scalar_t phase,
                                               scalar_t stanceTime) {
    // 1. Compute world velocity vector
    const scalar_t yaw_s = std::sin(basePose[YAW_IDX]);
    const scalar_t yaw_c = std::cos(basePose[YAW_IDX]);

    // express velocity in the world frame
    const scalar_t vx = vx_ * yaw_c - vy_ * yaw_s;
    const scalar_t vy = vx_ * yaw_s + vy_ * yaw_c;
    const vector3_t vWorld = (vector3_t() << vx, vy, 0.0).finished();

    // 3. compute desired foothold
    return phase * (hipPosition + (stanceTime / 2.0) * vWorld) + (1.0 - phase) * footPositionPrev;
}

void ReferenceGenerator::optimizeFoothold(vector3_t &nominalFoothold, vector6_t &nominalBasePose, const scalar_t time,
                                          const scalar_t currentPhase, bool firstTouchdown, size_t legIdx,
                                          int touchdownIdx, int liftOffIdx) {
    // Too late to optimize - use the last optimized foothold
    if (firstTouchdown && currentPhase > 0.5 && !firstRun_) {
        // diff = nextOptimizedFootholds_[legIdx] - nominalFoothold;
        nominalFoothold.head<2>() = nextOptimizedFootholds_[legIdx].head<2>();
        // nominalBasePose.head<2>() += diff.head<2>() / 4.0;
        return;
    }

    vector3_t diff = vector3_t::Zero();
    // Get gridmap
    const auto &map = gridMapInterface_.getMap();

    // Setup penalty function
    auto penaltyFunction = [this, touchdownIdx, liftOffIdx, legIdx](const Eigen::Vector3d &projectedPoint) {
        scalar_t cost = 0.0;
        // constexpr scalar_t w_kinematics_touchdown = 0.15;
        // constexpr scalar_t w_kinematics_liftoff = 0.14;
        constexpr scalar_t w_kinematics_touchdown = 0.05;
        constexpr scalar_t w_kinematics_liftoff = 0.02;
        const scalar_t dist1 = (projectedPoint - hipPositions_[touchdownIdx].col(legIdx)).norm();
        const scalar_t dist2 = (projectedPoint - hipPositions_[liftOffIdx].col(legIdx)).norm();
        cost += w_kinematics_touchdown * dist1;
        cost += w_kinematics_liftoff * dist2;

        if(dist1 >= 0.57 || dist2 >= 0.57) {
            cost += 1e4;
        }

        if(dist1 <= 0.1 || dist2 <= 0.1) {
            cost += 1e4;
        }
        
        return cost;
    };

    // Project nominal foothold onto the closest region
    const auto projection =
        getBestPlanarRegionAtPositionInWorld(nominalFoothold, gridMapInterface_.getPlanarRegions(), penaltyFunction);
    // diff = projection.positionInWorld - nominalFoothold;
    nominalFoothold = projection.positionInWorld;
    // nominalBasePose.head<2>() += diff.head<2>() / 4.0;

    if (firstTouchdown) {
        // Generate a convex set around the projected foothold
        const int numberOfVertices = 16;
        const double growthFactor = 1.05;
        const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
            projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numberOfVertices,
            growthFactor);

        // Store optimized foothold
        nextOptimizedFootholds_[legIdx] = nominalFoothold;

        // Store convex region
        firstContactInitialized_[legIdx] = true;
        firstContactTimes_[legIdx] = time;
        convexRegions_[legIdx] = convexRegion;
        convexRegionWorldTransforms_[legIdx] = projection.regionPtr->transformPlaneToWorld;
    }
}

void ReferenceGenerator::generateFootName2IndexMap() {
    for (size_t i = 0; i < modelSettings_.contactNames3DoF.size(); ++i) {
        footName2IndexMap_[modelSettings_.contactNames3DoF[i]] = i;
    }
}

}  // namespace legged_robot
}  // namespace ocs2