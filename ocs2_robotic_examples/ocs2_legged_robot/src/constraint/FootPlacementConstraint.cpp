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

#include <convex_plane_decomposition/PlanarRegion.h>

namespace ocs2 {
namespace legged_robot {

FootPlacementConstraint::FootPlacementConstraint(const SwitchedModelReferenceManager &referenceManager,
                                                 const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
                                                 size_t contactPointIndex, EndEffectorLinearConstraint::Config config)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      contactPointIndex_(contactPointIndex),
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 4, std::move(config))) {
        EndEffectorLinearConstraint::Config config2;
        config2.Ax = matrix_t::Zero(1, 3);
        config2.b = vector_t(1);

        config2.b[0] = 133;

        eeLinearConstraintPtr_->configure(config2);
      }

FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint &rhs)
    : StateConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      contactPointIndex_(rhs.contactPointIndex_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()) {}

bool FootPlacementConstraint::isActive(scalar_t time) const {
    bool initialized = referenceManagerPtr_->firstContactInitializedPtr->at(contactPointIndex_);
    if (!initialized) {
        return false;
    }

    const scalar_t firstContactTime = referenceManagerPtr_->firstContactTimesPtr->at(contactPointIndex_);
    if (time != firstContactTime) {
        return false;
    }

    setupFootholdPlacementConstraint();
    return true;
}

vector_t FootPlacementConstraint::getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const {
    return eeLinearConstraintPtr_->getValue(time, state, vector_t(), preComp);
}

VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(scalar_t time, const vector_t &state,
                                                                                  const PreComputation &preComp) const {
    return eeLinearConstraintPtr_->getLinearApproximation(time, state, vector_t(), preComp);
}

void FootPlacementConstraint::setupFootholdPlacementConstraint() const {
    auto &convexRegion = referenceManagerPtr_->convexRegionsPtr->at(contactPointIndex_);

    // Get number of constraints = number of vertices = number of polygon sides
    size_t nConstraints = convexRegion.size();

    // Setup config
    EndEffectorLinearConstraint::Config config;
    config.Ax = matrix_t::Zero(nConstraints, 3);
    config.b = vector_t(nConstraints);

    // Get convex region world transform
    const auto &convexRegionWorldTransform =
        referenceManagerPtr_->convexRegionWorldTransformsPtr->at(contactPointIndex_);

    // Setup 90 deg rotation matrix
    constexpr scalar_t c = std::cos(OCS2_PI / 2);
    constexpr scalar_t s = std::sin(OCS2_PI / 2);
    const matrix2_t R = (matrix2_t() << c, -s, s, c).finished();

    // Generate convex region linear constraint
    for (size_t i = 0; i < nConstraints; ++i) {
        size_t j = (i + 1) % nConstraints;

        // It doesn't make sense to do the transformation twice for each point -> cache it!
        const vector2_t point_i = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                                      convexRegion[i], convexRegionWorldTransform)
                                      .head<2>();
        const vector2_t point_j = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                                      convexRegion[j], convexRegionWorldTransform)
                                      .head<2>();

        // Vector from point_i to point_j
        const vector2_t s = point_j - point_i;

        // Normal vector
        const vector2_t n = R * s;

        config.Ax.block<1, 2>(i, 0) = n;
        config.b[i] = -n.dot(point_i);
    }

    eeLinearConstraintPtr_->configure(config);
}

}  // namespace legged_robot
}  // namespace ocs2
