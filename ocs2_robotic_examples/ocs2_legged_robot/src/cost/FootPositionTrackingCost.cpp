#include <pinocchio/fwd.hpp>

#include "ocs2_legged_robot/cost/FootPositionTrackingCost.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2 {
namespace legged_robot {

using ad_mat_t = Eigen::Matrix<ad_scalar_t, -1, -1>;

FootPositionTrackingCost::FootPositionTrackingCost(std::string name, matrix_t QPosition,
                                                   const std::string &libraryFolder, bool recompileLibraries,
                                                   bool verbose,
                                                   const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
                                                   const PinocchioInterfaceCppAd &pinocchioInterface,
                                                   const CentroidalModelInfo &centroidalModelInfo)
    : StateInputCostGaussNewtonAd(),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      QPosition_(std::move(QPosition)),
      pinocchioInterface_(pinocchioInterface),
      centroidalModelInfo_(centroidalModelInfo),
      mapping_(centroidalModelInfo_.toCppAd()),
      name(name) {

    // Set pinocchio interface
    mapping_.setPinocchioInterface(pinocchioInterface_);

    const size_t nParameters = 3;

    initialize(centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim, nParameters, name, libraryFolder,
               recompileLibraries, verbose);
}

vector_t FootPositionTrackingCost::getParameters(scalar_t time, const TargetTrajectories &trajectory,
                                                 const PreComputation &) const {
    const auto desiredState = trajectory.getDesiredState(time);
    return endEffectorKinematicsPtr_->getPosition(desiredState)[0];
}

ad_vector_t FootPositionTrackingCost::costVectorFunction(ad_scalar_t time, const ad_vector_t &state,
                                                         const ad_vector_t &input,
                                                         const ad_vector_t &parameters) {
    const ad_vector_t desiredPosition = parameters;

    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();

    const ad_vector_t q = mapping_.getPinocchioJointPosition(state);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    size_t frameId = model.getBodyId(endEffectorKinematicsPtr_->getIds()[0]);

    ad_vector_t currentPosition(3);
    currentPosition = data.oMf[frameId].translation();

    const ad_vector_t error = desiredPosition - currentPosition;
    const ad_mat_t QPositionAd = QPosition_.cast<ad_scalar_t>();
    return QPositionAd * error;
}

FootPositionTrackingCost *FootPositionTrackingCost::FootPositionTrackingCost::clone() const {
    return new FootPositionTrackingCost(
        name, QPosition_, "tmp/ocs2/" + name, false, false, *endEffectorKinematicsPtr_, pinocchioInterface_,
        centroidalModelInfo_
    );     
}

}  // namespace legged_robot
}  // namespace ocs2