#include "ocs2_legged_robot_ros/reference_generator/AnymalInverseKinematics.h"

#include <ros/ros.h>
#include <boost/property_tree/info_parser.hpp>

namespace ocs2 {
namespace legged_robot {

AnymalInverseKinematics::AnymalInverseKinematics(const PinocchioInterface &pinocchioInterface,
                                                 const ModelSettings &modelSettings, const std::string &configFile)
    : geometry_(configFile), pinocchioInterface_(pinocchioInterface), modelSettings_(modelSettings) {
    modelSettings_.contactNames3DoF = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};
    generateFootName2IndexMap();
    generateHipShiftSignMap();
}

void AnymalInverseKinematics::generateFootName2IndexMap() {
    for (size_t i = 0; i < modelSettings_.contactNames3DoF.size(); ++i) {
        footName2IndexMap_[modelSettings_.contactNames3DoF[i]] = i;
    }
}

void AnymalInverseKinematics::generateHipShiftSignMap() {
    legIndex2ShiftSign_[footName2Index("LF_FOOT")] = std::make_pair(1.0, 1.0);
    legIndex2ShiftSign_[footName2Index("LH_FOOT")] = std::make_pair(-1.0, 1.0);
    legIndex2ShiftSign_[footName2Index("RF_FOOT")] = std::make_pair(1.0, -1.0);
    legIndex2ShiftSign_[footName2Index("RH_FOOT")] = std::make_pair(-1.0, -1.0);
}

pinocchio::SE3 AnymalInverseKinematics::getBaseWorldPose(const vector3_t &basePositionWorld,
                                                         const vector3_t &baseOrientationWorldYPR) {
    // 1. Generate rotation matrix from YPR angles
    const auto R = pinocchio::rpy::rpyToMatrix(baseOrientationWorldYPR.reverse());

    // 2. Generate base pose
    return pinocchio::SE3(R, basePositionWorld);
}

vector3_t AnymalInverseKinematics::convertLegPositionWorldToBase(const vector3_t &legPositionWorld,
                                                                 const pinocchio::SE3 &basePoseWorld,
                                                                 const std::string &legName) {
    // 1. Generate shift vector from base to hip
    const auto shiftSign = footName2ShiftSign(legName);
    const vector3_t shift_vector = {shiftSign.first * geometry_.body_length / 2.0,
                                    shiftSign.second * geometry_.body_width / 2.0, 0.0};

    // 2. Compute foot position in the hip frame (with the orientation of the base frame)
    return basePoseWorld.actInv_impl(legPositionWorld) - shift_vector;
}

vector3_t AnymalInverseKinematics::singleLegInverseKinematics(const vector3_t &legPositionWorld,
                                                              const pinocchio::SE3 &baseWorldPose,
                                                              const std::string &legName) {
    // 1. Get leg index
    const size_t legIndex = footName2Index(legName);

    // 2. Get foot position in expressed in hip frame
    const vector3_t legPositionBase = convertLegPositionWorldToBase(legPositionWorld, baseWorldPose, legName);

    // 3. Unpack position
    const scalar_t x = legPositionBase[0];
    const scalar_t y = legPositionBase[1];
    const scalar_t z = legPositionBase[2];

    // 4. Unpack geometry
    const scalar_t d2 = legIndex >= 2 ? -geometry_.l2 : geometry_.l2;
    const scalar_t a3 = geometry_.l3;
    const scalar_t a4 = geometry_.l4;

    // 5. Compute theta1
    const scalar_t E = y * y + z * z - d2 * d2;
    const scalar_t E_sqrt = sqrt(E);
    const scalar_t theta1 = atan2(E_sqrt, d2) + atan2(z, y);

    // 6. Compute theta4
    double D = (E + x * x - a3 * a3 - a4 * a4) / (2.0 * a3 * a4);
    D = std::max(-1.0, std::min(1.0, D));
    if (D == 1.0 || D == -1.0) {
        ROS_WARN_THROTTLE(1, "Foot position out of reach!");
    }
    double theta4 = -atan2(sqrt(1.0 - D * D), D);
    constexpr scalar_t theta4_offset = 0.254601;
    double theta4_final = theta4 + theta4_offset;
    if (legIndex % 2 == 1) {  // hind legs
        theta4 = -theta4;
        theta4_final = -theta4_final;
    }

    // 7. Compute theta3
    double theta3 = atan2(-x, E_sqrt) - atan2(a4 * sin(theta4), a3 + a4 * cos(theta4));

    return {theta1, theta3, theta4_final};
}

vector_t AnymalInverseKinematics::anymalInverseKinematics(const vector3_t &bodyPosition,
                                                          const vector3_t &bodyOrientationYPR,
                                                          const std::array<vector3_t, 4> &legPositionsWorld) {
    // 1. Compute base world pose
    const pinocchio::SE3 basePoseWorld = getBaseWorldPose(bodyPosition, bodyOrientationYPR);

    // 2. Compute inverse kinematics for individual legs
    vector_t jointAngles(12);
    for (size_t i = 0; i < modelSettings_.contactNames3DoF.size(); ++i) {
        jointAngles.segment<3>(3 * i) =
            singleLegInverseKinematics(legPositionsWorld[i], basePoseWorld, modelSettings_.contactNames3DoF[i]);
    }

    return jointAngles;
}

/**********************************************************************************************************************/

AnymalGeometry::AnymalGeometry(const std::string &configFile) {
    // 1. Read config file
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(configFile, pt);

    // 2. Retrieve geometry parameters
    const scalar_t body_length = readValueFromPropertyTree(pt, "body_length");
    const scalar_t body_width = readValueFromPropertyTree(pt, "body_width");
    const scalar_t l1 = readValueFromPropertyTree(pt, "l1");
    const scalar_t l2 = readValueFromPropertyTree(pt, "l2");
    const scalar_t l3 = readValueFromPropertyTree(pt, "l3");
    const scalar_t l4 = readValueFromPropertyTree(pt, "l4");

    // 3. Set geometry parameters
    setBodyDimensions(body_length, body_width);
    setLegDimensions(l1, l2, l3, l4);

    // 4. Check if all the parameters are set
    if (!isInitialized()) {
        ROS_ERROR("AnymalGeometry: Failed to initialize from config file: %s", configFile.c_str());
    }
}

void AnymalGeometry::setBodyDimensions(scalar_t body_length, scalar_t body_width) {
    this->body_length = body_length;
    this->body_width = body_width;
}

void AnymalGeometry::setLegDimensions(scalar_t l1, scalar_t l2, scalar_t l3, scalar_t l4) {
    this->l1 = l1;
    this->l2 = l2;
    this->l3 = l3;
    this->l4 = l4;
}

bool AnymalGeometry::isInitialized() const {
    return (body_length >= 0 && body_width >= 0 && l1 >= 0 && l2 >= 0 && l3 >= 0 && l4 >= 0);  // all the values are set
}

scalar_t AnymalGeometry::readValueFromPropertyTree(const boost::property_tree::ptree &pt, const std::string &key,
                                                   scalar_t defaultValue) {
    scalar_t value;
    try {
        value = pt.get<scalar_t>(key);
    } catch (const boost::property_tree::ptree_bad_path &) {
        value = defaultValue;
    }
    return value;
}

}  // namespace legged_robot
}  // namespace ocs2