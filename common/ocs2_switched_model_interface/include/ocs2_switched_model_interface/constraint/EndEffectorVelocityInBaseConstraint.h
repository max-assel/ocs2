#pragma once

#include <ocs2_switched_model_interface/constraint/EndEffectorConstraint.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h>

namespace switched_model {

class EndEffectorVelocityInBaseConstraint : public EndEffectorConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = switched_model::EndEffectorConstraint;
  using typename BASE::ad_com_model_t;
  using typename BASE::ad_interface_t;
  using typename BASE::ad_kinematic_model_t;
  using typename BASE::ad_scalar_t;
  using typename BASE::ad_vector_t;
  using typename BASE::constraint_timeStateInput_matrix_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::LinearApproximation_t;
  using typename BASE::QuadraticApproximation_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;
  using typename BASE::timeStateInput_matrix_t;
  using settings_t = EndEffectorVelocityConstraintSettings;

  explicit EndEffectorVelocityInBaseConstraint(int legNumber, settings_t settings, ad_com_model_t& adComModel,
                                               ad_kinematic_model_t& adKinematicsModel, bool generateModels = true,
                                               std::string constraintPrefix = "b_EEVelocityConstraint_")
      : BASE(ConstraintOrder::Linear, std::move(constraintPrefix), legNumber, std::move(settings), adComModel, adKinematicsModel,
             EndEffectorVelocityInBaseConstraint::adfunc, generateModels) {}

  EndEffectorVelocityInBaseConstraint(const EndEffectorVelocityInBaseConstraint& rhs) = default;

  EndEffectorVelocityInBaseConstraint* clone() const override { return new EndEffectorVelocityInBaseConstraint(*this); }

 private:
  static void adfunc(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber, const ad_vector_t& tapedInput,
                     ad_vector_t& b_footVelocity) {
    // Extract elements from taped input
    ad_scalar_t t = tapedInput(0);
    comkino_state_ad_t x = tapedInput.segment(1, STATE_DIM);
    comkino_input_ad_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

    // Extract elements from state
    const base_coordinate_ad_t baseLocalVelocities = getComLocalVelocities(x);
    const joint_coordinate_ad_t qJoints = getJointPositions(x);
    const joint_coordinate_ad_t dqJoints = getJointVelocities(u);

    b_footVelocity = adKinematicsModel.footVelocityInBaseFrame(legNumber, baseLocalVelocities, qJoints, dqJoints);
  }
};
}  // namespace switched_model
