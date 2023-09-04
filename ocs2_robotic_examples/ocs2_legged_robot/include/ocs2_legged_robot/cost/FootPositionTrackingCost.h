#pragma once

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

namespace ocs2 {
namespace legged_robot {

class FootPositionTrackingCost final : public StateInputCostGaussNewtonAd {
   public:
    /**
     * Constructor
     */
    FootPositionTrackingCost(matrix_t QPosition, const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
                             const PinocchioInterfaceCppAd &pinocchioInterface,
                             const CentroidalModelInfo &centroidalModelInfo,
                             const std::string &modelName,
                             const std::string& modelFolder = "/tmp/ocs2", 
                             bool recompileLibraries = true, bool verbose = true);

    /**
     * Copy constructor
     */
    FootPositionTrackingCost(const FootPositionTrackingCost &rhs);

    ~FootPositionTrackingCost() override = default;

    /** Get the parameter vector */
    vector_t getParameters(scalar_t time, const TargetTrajectories &desiredTrajectory,
                           const PreComputation &) const override;

   protected:
    ad_vector_t costVectorFunction(ad_scalar_t time, const ad_vector_t &state, const ad_vector_t &input,
                                   const ad_vector_t &desiredPosition) override;

    FootPositionTrackingCost *clone() const override { 
        return new FootPositionTrackingCost(*this); }

   private:
    std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
    PinocchioInterfaceCppAd pinocchioInterface_;
    CentroidalModelPinocchioMappingTpl<ad_scalar_t> mapping_;

    // Position cost matrix
    matrix_t QPosition_;
};

}  // namespace legged_robot
}  // namespace ocs2