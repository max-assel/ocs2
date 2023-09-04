#pragma once

#include <memory>
#include <string>
#include <utility>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"
#include "ocs2_legged_robot/constraint/SdfDistanceTransformInterface.h"

namespace ocs2 {
namespace legged_robot {

/**
 * End-effector distance constraint Function.
 */
class EndEffectorDistanceConstraint final : public ocs2::StateConstraint {
   public:
    /** Constructor */
    EndEffectorDistanceConstraint(const SwitchedModelReferenceManager &referenceManager, size_t stateDim,
                                  bool alwaysActive, const EndEffectorKinematics<scalar_t> &kinematics, SdfDistanceTransformInterface &distanceTransform,
                                  size_t contactPointIndex = 0);

    /** Default destructor */
    ~EndEffectorDistanceConstraint() override = default;

    void set(SdfDistanceTransformInterface &distanceTransform) { distanceTransformPtr_ = &distanceTransform; }

    EndEffectorDistanceConstraint *clone() const override { return new EndEffectorDistanceConstraint(*this); }

    bool isActive(scalar_t time) const override;

    size_t getNumConstraints(scalar_t time) const override { return kinematicsPtr_->getIds().size(); }
    const std::vector<std::string> &getIDs() const { return kinematicsPtr_->getIds(); }

    vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                             const PreComputation &preComp) const override;

    size_t getStateDimensions() const { return stateDim_; }
    EndEffectorKinematics<scalar_t> &getKinematics() { return *kinematicsPtr_; }

   private:
    EndEffectorDistanceConstraint(const EndEffectorDistanceConstraint &other);

    const size_t stateDim_;
    const scalar_t weight_;
    const bool alwaysActive_;

    const SwitchedModelReferenceManager *referenceManagerPtr_;
    std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr_;

    size_t contactPointIndex_;

    SdfDistanceTransformInterface *distanceTransformPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2