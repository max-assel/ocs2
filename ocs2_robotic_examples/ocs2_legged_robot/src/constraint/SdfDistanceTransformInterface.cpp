#include "ocs2_legged_robot/constraint/SdfDistanceTransformInterface.h"

namespace ocs2 {

scalar_t SdfDistanceTransformInterface::getValue(const vector3_t &p) const { return sdfPtr_->value(p); }

std::pair<scalar_t, vector3_t> SdfDistanceTransformInterface::getLinearApproximation(const vector3_t &p) const {
    return sdfPtr_->valueAndDerivative(p);
}

}  // namespace ocs2
