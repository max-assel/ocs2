#include <pinocchio/fwd.hpp>

#include "ocs2_legged_robot/cost/FootPositionTrackingCost.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2 {
namespace legged_robot {

using ad_mat_t = Eigen::Matrix<ad_scalar_t, -1, -1>;

FootPositionTrackingCost::FootPositionTrackingCost(matrix_t QPosition,
                                                   const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
                                                   const PinocchioInterfaceCppAd &pinocchioInterface,
                                                   const CentroidalModelInfo &centroidalModelInfo,
                                                   const std::string &modelName, const std::string &modelFolder,
                                                   bool recompileLibraries, bool verbose)
    : StateInputCostGaussNewtonAd(),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      QPosition_(QPosition),
      pinocchioInterface_(pinocchioInterface),
      mapping_(centroidalModelInfo.toCppAd()) {
    
    constexpr size_t nParameters = 3;

    initialize(centroidalModelInfo.stateDim, centroidalModelInfo.inputDim, nParameters, modelName, modelFolder,
               recompileLibraries, verbose);
}

vector_t FootPositionTrackingCost::getParameters(scalar_t time, const TargetTrajectories &trajectory,
                                                 const PreComputation &) const {
    const auto desiredState = trajectory.getDesiredState(time);
    return endEffectorKinematicsPtr_->getPosition(desiredState)[0];
}

ad_vector_t FootPositionTrackingCost::costVectorFunction(ad_scalar_t time, const ad_vector_t &state,
                                                         const ad_vector_t &input,
                                                         const ad_vector_t &desiredPosition) {
    // TODO: Because of the constness of this function, I can't use the pinocchioInterface_ directly
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();

    // Get pinocchio joint position
    const ad_vector_t q = mapping_.getPinocchioJointPosition(state);

    // Compute forward kinematics
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    // Get end-effector position
    const size_t frameId = model.getBodyId(endEffectorKinematicsPtr_->getIds()[0]);
    ad_vector_t currentPosition = data.oMf[frameId].translation();

    // Compute position error
    const ad_vector_t error = desiredPosition - currentPosition;

    // Return cost
    return QPosition_.cast<ad_scalar_t>() * error;
}

FootPositionTrackingCost::FootPositionTrackingCost(const FootPositionTrackingCost &rhs)
    : StateInputCostGaussNewtonAd(rhs),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      QPosition_(rhs.QPosition_),
      pinocchioInterface_(rhs.pinocchioInterface_),
      mapping_(rhs.mapping_.getCentroidalModelInfo()) {}

}  // namespace legged_robot
}  // namespace ocs2