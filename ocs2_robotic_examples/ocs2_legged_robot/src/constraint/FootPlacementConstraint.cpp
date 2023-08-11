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
                                                 size_t contactPointIndex, EndEffectorLinearConstraint::Config config)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      contactPointIndex_(contactPointIndex),
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 4, std::move(config))) {}

bool FootPlacementConstraint::isActive(scalar_t time) const {
    constexpr scalar_t eps = 1e-7;
    bool newContact = !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_] &&
                      referenceManagerPtr_->getContactFlags(time + eps)[contactPointIndex_];

    if (newContact) {
        updateConfig(time);
    }
    return newContact;
}

vector_t FootPlacementConstraint::getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const {
    return eeLinearConstraintPtr_->getValue(time, state, vector_t(), preComp);
}

FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint &rhs)
    : StateConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      contactPointIndex_(rhs.contactPointIndex_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()) {}

VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(scalar_t time, const vector_t &state,
                                                                                  const PreComputation &preComp) const {
    return eeLinearConstraintPtr_->getLinearApproximation(time, state, vector_t(), preComp);
}

void FootPlacementConstraint::updateConfig(scalar_t time) const {
    const auto &desiredState = referenceManagerPtr_->getTargetTrajectories().getDesiredState(time);
    auto &endEffectorKinematics = eeLinearConstraintPtr_->getEndEffectorKinematics();
    const vector3_t desiredFoothold = endEffectorKinematics.getPosition(desiredState)[0];
    const scalar_t x = desiredFoothold[0];

    constexpr scalar_t terrainGap = 0.06;
    constexpr scalar_t platformWidth = 3.0;

    // Find the closest terrain point to the desired foothold
    scalar_t idx = std::ceil(x / (terrainGap + platformWidth));

    // Find closer terrain point
    scalar_t xTerrain = idx * (terrainGap + platformWidth);
    scalar_t xPlatform = xTerrain - terrainGap;

    scalar_t x_lower, x_upper;
    if (x - xPlatform < 0.0) {
        x_lower = xPlatform - platformWidth;
        x_upper = xPlatform;
    } else {
        if (std::abs(x - xPlatform) <= std::abs(x - xTerrain)) {
            x_lower = xPlatform - platformWidth;
            x_upper = xPlatform;
        } else {
            x_lower = xTerrain;
            x_upper = xTerrain + platformWidth;
        }
    }

    eeLinearConstraintPtr_->configure(getConfig(x_lower, x_upper));
}

EndEffectorLinearConstraint::Config FootPlacementConstraint::getConfig(scalar_t x_lower, scalar_t x_upper) const {
    EndEffectorLinearConstraint::Config config;
    config.b = (vector_t(2) << -x_lower, x_upper).finished();
    config.Ax = matrix_t(2, 3);
    config.Ax << 1.0, 0.0, 0.0, -1.0, 0.0, 0.0;
    return config;
}

}  // namespace legged_robot
}  // namespace ocs2
