#pragma once

#include "ocs2_perceptive/distance_transform/DistanceTransformInterface.h"
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace ocs2 {

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

class SdfDistanceTransformInterface : public DistanceTransformInterface {
   public:
    SdfDistanceTransformInterface() = default;

    inline void setSdfPtr(grid_map::SignedDistanceField *sdfPtr) { sdfPtr_ = sdfPtr; }

    scalar_t getValue(const vector3_t &p) const override;
    vector3_t getProjectedPoint(const vector3_t &p) const override {return vector3_t::Zero();} // Not implemented
    std::pair<scalar_t, vector3_t> getLinearApproximation(const vector3_t &) const override;

   public:
    grid_map::SignedDistanceField *sdfPtr_;
};

}  // namespace ocs2