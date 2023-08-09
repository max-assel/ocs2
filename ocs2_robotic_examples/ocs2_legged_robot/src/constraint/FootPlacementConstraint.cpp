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

#include "ocs2_legged_robot/constraint/FootPlacementConstraint.h"

namespace ocs2 {
namespace legged_robot {

FootPlacementConstraint::FootPlacementConstraint(const SwitchedModelReferenceManager &referenceManager,
                                                 const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
                                                 EndEffectorLinearConstraint::Config config)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 4, std::move(config))) {
    updateConfig(vector_t());
}

vector_t FootPlacementConstraint::getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const {
    return eeLinearConstraintPtr_->getValue(time, state, vector_t(), preComp);
}

VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(scalar_t time, const vector_t &state,
                                                                                  const PreComputation &preComp) const {
    return eeLinearConstraintPtr_->getLinearApproximation(time, state, vector_t(), preComp);
}

void FootPlacementConstraint::updateConfig(const vector_t &state) {
    eeLinearConstraintPtr_->configure(getConfig(state));
}

EndEffectorLinearConstraint::Config FootPlacementConstraint::getConfig(const vector_t &state) {
    EndEffectorLinearConstraint::Config config;
    // +x -x +y -y
    config.b = (vector_t(4) << 1.0, 1.0, 1.0, 1.0).finished();
    config.Ax = matrix_t(4, 3);
    config.Ax << 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0;
    return config;
}

}  // namespace legged_robot
}  // namespace ocs2
