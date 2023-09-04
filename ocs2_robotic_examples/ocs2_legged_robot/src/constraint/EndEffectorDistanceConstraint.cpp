#include "ocs2_legged_robot/constraint/EndEffectorDistanceConstraint.h"

#include "ocs2_legged_robot/gait/LegLogic.h"

namespace ocs2 {
namespace legged_robot {

constexpr scalar_t CLEARANCE = 0.05;

EndEffectorDistanceConstraint::EndEffectorDistanceConstraint(const SwitchedModelReferenceManager &referenceManager,
                                                             size_t stateDim, bool alwaysActive,
                                                             const EndEffectorKinematics<scalar_t> &kinematics,
                                                             SdfDistanceTransformInterface &distanceTransform,
                                                             size_t contactPointIndex)
    : StateConstraint(ConstraintOrder::Linear),
      stateDim_(stateDim),
      weight_(1.0),
      alwaysActive_(alwaysActive),
      referenceManagerPtr_(&referenceManager),
      kinematicsPtr_(kinematics.clone()),
      contactPointIndex_(contactPointIndex),
      distanceTransformPtr_(&distanceTransform) {
    if (kinematicsPtr_->getIds().size() != 1) {
        throw std::runtime_error("EndEffectorDistanceConstraint: Only one end-effector is supported.");
    }
}

EndEffectorDistanceConstraint::EndEffectorDistanceConstraint(const EndEffectorDistanceConstraint &other)
    : StateConstraint(other),
      stateDim_(other.stateDim_),
      kinematicsPtr_(other.kinematicsPtr_->clone()),
      alwaysActive_(other.alwaysActive_),
      weight_(other.weight_),
      referenceManagerPtr_(other.referenceManagerPtr_),
      contactPointIndex_(other.contactPointIndex_),
      distanceTransformPtr_(other.distanceTransformPtr_) {}

bool EndEffectorDistanceConstraint::isActive(scalar_t time) const {
    if (alwaysActive_) {
        return true;
    }

    bool active = !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
    if (active) {
        const auto &modeSchedule = referenceManagerPtr_->getModeSchedule();
        auto phase = getSwingPhasePerLeg(time, modeSchedule)[contactPointIndex_].phase;
        if (phase <= 0.3 || phase >= 0.7) {
            active = false;
        }
    }

    if (active) {
        distanceTransformPtr_->setSdfPtr(referenceManagerPtr_->sdfPtr);
    }
    return active;
}

vector_t EndEffectorDistanceConstraint::getValue(scalar_t time, const vector_t &state,
                                                 const PreComputation &preComp) const {
    if (distanceTransformPtr_ == nullptr) {
        throw std::runtime_error("[EndEffectorDistanceConstraint] First, set the distance-transform by calling set()!");
    }

    const auto numEEs = kinematicsPtr_->getIds().size();
    vector_t g(numEEs);

    scalar_t weight = 1.0;

    const auto position = kinematicsPtr_->getPosition(state)[0];
    g[0] = weight * (distanceTransformPtr_->getValue(position) - CLEARANCE);

    return g;
}

VectorFunctionLinearApproximation EndEffectorDistanceConstraint::getLinearApproximation(
    scalar_t time, const vector_t &state, const PreComputation &preComp) const {
    if (distanceTransformPtr_ == nullptr) {
        throw std::runtime_error("[EndEffectorDistanceConstraint] First, set the distance-transform by calling set()!");
    }

    // distanceTransformPtr_->setSdfPtr(referenceManagerPtr_->sdfPtr);

    const auto numEEs = kinematicsPtr_->getIds().size();
    const auto eePosLinApprox = kinematicsPtr_->getPositionLinearApproximation(state);

    scalar_t weight = 1.0;

    VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numEEs, stateDim_, 0);
    const auto distanceValueGradient = distanceTransformPtr_->getLinearApproximation(eePosLinApprox[0].f);
    approx.f[0] = weight * (distanceValueGradient.first - CLEARANCE);
    approx.dfdx.row(0).noalias() = weight * (distanceValueGradient.second.transpose() * eePosLinApprox[0].dfdx);

    return approx;
}

}  // namespace legged_robot
}  // namespace ocs2
