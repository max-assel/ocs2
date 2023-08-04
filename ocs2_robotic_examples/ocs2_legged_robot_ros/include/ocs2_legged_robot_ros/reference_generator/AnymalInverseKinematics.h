#pragma once

#include <pinocchio/fwd.hpp>

// std-lib
#include <utility>  // std::pair
#include <vector>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>

// ocs2
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>

// pinocchio
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/math/rpy.hpp>

namespace ocs2 {
namespace legged_robot {

constexpr scalar_t UNINITIALIZED = -1.0;

struct AnymalGeometry {
    // Body dimensions
    scalar_t body_length;
    scalar_t body_width;

    // Leg dimensions
    scalar_t l1;
    scalar_t l2;
    scalar_t l3;
    scalar_t l4;

    // Constructor
    AnymalGeometry(const std::string &configFile);

    // Helper functions
    void setBodyDimensions(scalar_t body_length, scalar_t body_width);
    void setLegDimensions(scalar_t l1, scalar_t l2, scalar_t l3, scalar_t l4);
    bool isInitialized() const;
    scalar_t readValueFromPropertyTree(const boost::property_tree::ptree &pt, const std::string &key,
                                       scalar_t defaultValue = UNINITIALIZED);
};

class AnymalInverseKinematics {
   public:
    // Constructor
    AnymalInverseKinematics(const PinocchioInterface &pinocchioInterface, const ModelSettings &modelSettings,
                            const std::string &configFile);

    // Compute inverse kinematics for a single leg
    vector3_t singleLegInverseKinematics(const vector3_t &legPositionWorld, const pinocchio::SE3 &baseWorldPose,
                                         const std::string &legName);

    // Compute inverse kinematics for the entire robot
    vector_t anymalInverseKinematics(const vector3_t &bodyPosition, const vector3_t &bodyOrientation,
                                     const std::array<vector3_t, 4> &legPositionsWorld);

    // Convert base position (XYZ coordinates) and orientation (RPY angles) to
    // base pose expressed in the world frame
    pinocchio::SE3 getBaseWorldPose(const vector3_t &basePositionWorld, const vector3_t &baseOrientationWorldYPR);

    // Return a reference to the 'PinocchioInterface' object
    inline PinocchioInterface &getPinocchioInterface() { return pinocchioInterface_; }

    // Convert leg position expressed in the world frame to the hip frame with
    // the orientation of the base frame
    vector3_t convertLegPositionWorldToBase(const vector3_t &legPositionWorld, const pinocchio::SE3 &basePoseWorld,
                                            const std::string &legName);

   private:
    // Helper functions
    void generateFootName2IndexMap();
    void generateHipShiftSignMap();
    inline size_t footName2Index(const std::string &footName) { return footName2IndexMap_[footName]; }
    inline size_t footName2Index(const char *footName) { return footName2IndexMap_[std::string(footName)]; }
    inline std::pair<scalar_t, scalar_t> footName2ShiftSign(const std::string &legName) {
        return legIndex2ShiftSign_[footName2Index(legName)];
    }
    inline std::pair<scalar_t, scalar_t> Index2ShiftSign(size_t legIndex) { return legIndex2ShiftSign_[legIndex]; }

    const AnymalGeometry geometry_;
    PinocchioInterface pinocchioInterface_;
    ModelSettings modelSettings_;
    std::unordered_map<std::string, size_t> footName2IndexMap_;
    std::array<std::pair<scalar_t, scalar_t>, 4> legIndex2ShiftSign_;
};

}  // namespace legged_robot
}  // namespace ocs2