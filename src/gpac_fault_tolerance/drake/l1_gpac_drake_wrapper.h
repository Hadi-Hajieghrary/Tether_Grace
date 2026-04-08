#pragma once

#include <drake/systems/framework/leaf_system.h>

#include "gpac_fault_tolerance/adapters/gpac_seam_adapter.h"

namespace tether_grace::gpac_fault_tolerance::drake {

class L1GpacDrakeWrapper final : public ::drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(L1GpacDrakeWrapper);

  explicit L1GpacDrakeWrapper(const adapters::GpacSeamAdapterParams &params =
                                  adapters::GpacSeamAdapterParams(),
                              double nominal_update_period_seconds = 0.005);

  const ::drake::systems::InputPort<double> &get_position_input_port() const;
  const ::drake::systems::InputPort<double> &get_velocity_input_port() const;
  const ::drake::systems::InputPort<double> &
  get_commanded_acceleration_input_port() const;

  const ::drake::systems::OutputPort<double> &
  get_controller_disturbance_output_port() const;
  const ::drake::systems::OutputPort<double> &
  get_filtered_compensation_output_port() const;
  const ::drake::systems::OutputPort<double> &
  get_safety_lower_bound_output_port() const;
  const ::drake::systems::OutputPort<double> &
  get_safety_upper_bound_output_port() const;
  const ::drake::systems::OutputPort<double> &get_raw_sigma_output_port() const;
  const ::drake::systems::OutputPort<double> &
  get_prediction_error_output_port() const;

private:
  ::drake::systems::EventStatus
  UpdateAdapter(const ::drake::systems::Context<double> &context,
                ::drake::systems::DiscreteValues<double> *discrete_state) const;

  void SetDefaultState(const ::drake::systems::Context<double> &context,
                       ::drake::systems::State<double> *state) const override;

  void CalcControllerDisturbance(
      const ::drake::systems::Context<double> &context,
      ::drake::systems::BasicVector<double> *output) const;
  void
  CalcFilteredCompensation(const ::drake::systems::Context<double> &context,
                           ::drake::systems::BasicVector<double> *output) const;
  void
  CalcSafetyLowerBound(const ::drake::systems::Context<double> &context,
                       ::drake::systems::BasicVector<double> *output) const;
  void
  CalcSafetyUpperBound(const ::drake::systems::Context<double> &context,
                       ::drake::systems::BasicVector<double> *output) const;
  void CalcRawSigma(const ::drake::systems::Context<double> &context,
                    ::drake::systems::BasicVector<double> *output) const;
  void CalcPredictionError(const ::drake::systems::Context<double> &context,
                           ::drake::systems::BasicVector<double> *output) const;

  adapters::GpacSeamAdapterParams params_;
  double update_period_seconds_;

  int position_port_{};
  int velocity_port_{};
  int commanded_acceleration_port_{};
  int controller_disturbance_port_{};
  int filtered_compensation_port_{};
  int safety_lower_bound_port_{};
  int safety_upper_bound_port_{};
  int raw_sigma_port_{};
  int prediction_error_port_{};
  int discrete_state_index_{};

  static constexpr int kPredictedPositionOffset = 0;
  static constexpr int kPredictedVelocityOffset = 3;
  static constexpr int kCompensationOffset = 6;
  static constexpr int kSigmaOffset = 9;
  static constexpr int kPredictionErrorOffset = 12;
  static constexpr int kStateSize = 15;
};

} // namespace tether_grace::gpac_fault_tolerance::drake
