/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"

namespace switched_model {

class SwitchedModelCostBase : public ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM> BASE;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  using logic_rules_t = SwitchedModelLogicRulesBase;

  using com_model_t = ComModelBase<double>;

  //! Constructor
  SwitchedModelCostBase(const com_model_t& comModel, std::shared_ptr<const logic_rules_t> logicRulesPtr, const state_matrix_t& Q,
                        const input_matrix_t& R, const state_matrix_t& QFinal);

  //! Copy constructor
  SwitchedModelCostBase(const SwitchedModelCostBase& rhs);

  //! Destructor
  ~SwitchedModelCostBase() override = default;

  //! clone SwitchedModelCostBase class.
  SwitchedModelCostBase* clone() const override;

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

 private:
  void inputFromContactFlags(contact_flag_t contactFlags, dynamic_vector_t& inputs);

  std::unique_ptr<com_model_t> comModelPtr_;

  std::shared_ptr<const logic_rules_t> logicRulesPtr_;
};

}  // end of namespace switched_model
