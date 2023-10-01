/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>

#include "ocs2_legged_robot/constraint/EndEffectorLinearConstraint.h"
#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

#define OCS2_PI 3.14159265358979323846

namespace ocs2 {
namespace legged_robot {

using matrix2_t = Eigen::Matrix<scalar_t, 2, 2>;
using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;

class FootPlacementConstraint final : public StateConstraint {
   public:
    FootPlacementConstraint(const SwitchedModelReferenceManager &referenceManager,
                            const EndEffectorKinematics<scalar_t> &endEffectorKinematics, size_t contactPointIndex,
                            EndEffectorLinearConstraint::Config config = EndEffectorLinearConstraint::Config());

    ~FootPlacementConstraint() override = default;
    FootPlacementConstraint *clone() const override { return new FootPlacementConstraint(*this); }
    size_t getNumConstraints(scalar_t time) const override { return referenceManagerPtr_->convexRegionsPtr->at(contactPointIndex_).size(); }
    // size_t getNumConstraints(scalar_t time) const override { return 1; }
    bool isActive(scalar_t time) const override;

    vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                             const PreComputation &preComp) const override;

   private:
    void setupFootholdPlacementConstraint() const;
    FootPlacementConstraint(const FootPlacementConstraint &rhs);

    const SwitchedModelReferenceManager *referenceManagerPtr_;
    std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
    size_t contactPointIndex_;
};

}  // namespace legged_robot
}  // namespace ocs2