#include "gpac_fault_tolerance/drake/l1_gpac_drake_wrapper.h"

#include <drake/systems/framework/basic_vector.h>

namespace tether_grace::gpac_fault_tolerance::drake {

namespace {

constexpr int kPredictedPositionOffset = 0;
constexpr int kPredictedVelocityOffset = 3;
constexpr int kCompensationOffset = 6;
constexpr int kSigmaOffset = 9;
constexpr int kPredictionErrorOffset = 12;

adapters::Vector3 ReadVector3(const ::drake::systems::Context<double> &context,
                              const ::drake::systems::InputPort<double> &port) {
  const auto &vector = port.Eval(context);
  return {vector[0], vector[1], vector[2]};
}

void WriteVector3(const adapters::Vector3 &values,
                  ::drake::systems::BasicVector<double> *output) {
  output->SetAtIndex(0, values[0]);
  output->SetAtIndex(1, values[1]);
  output->SetAtIndex(2, values[2]);
}

::tether_grace::gpac_fault_tolerance::L1AdaptiveState
ReadStoredState(const ::drake::systems::Context<double> &context,
                int discrete_state_index) {
  const auto &values = context.get_discrete_state(discrete_state_index).value();

  ::tether_grace::gpac_fault_tolerance::L1AdaptiveState state{};
  for (int axis = 0; axis < 3; ++axis) {
    state.predicted_position[axis] = values[kPredictedPositionOffset + axis];
    state.predicted_velocity[axis] = values[kPredictedVelocityOffset + axis];
    state.compensation[axis] = values[kCompensationOffset + axis];
    state.sigma_hat[axis] = values[kSigmaOffset + axis];
    state.prediction_error[axis] = values[kPredictionErrorOffset + axis];
  }
  return state;
}

void WriteStoredState(
    const ::tether_grace::gpac_fault_tolerance::L1AdaptiveState &state,
    ::drake::systems::BasicVector<double> *values_vector) {
  auto values = values_vector->get_mutable_value();
  values.setZero();
  for (int axis = 0; axis < 3; ++axis) {
    values[kPredictedPositionOffset + axis] = state.predicted_position[axis];
    values[kPredictedVelocityOffset + axis] = state.predicted_velocity[axis];
    values[kCompensationOffset + axis] = state.compensation[axis];
    values[kSigmaOffset + axis] = state.sigma_hat[axis];
    values[kPredictionErrorOffset + axis] = state.prediction_error[axis];
  }
}

adapters::Vector3
ReadCompensation(const ::drake::systems::Context<double> &context,
                 int discrete_state_index) {
  const auto &values = context.get_discrete_state(discrete_state_index).value();
  return {
      values[kCompensationOffset + 0],
      values[kCompensationOffset + 1],
      values[kCompensationOffset + 2],
  };
}

adapters::Vector3 ReadSigmaHat(const ::drake::systems::Context<double> &context,
                               int discrete_state_index) {
  const auto &values = context.get_discrete_state(discrete_state_index).value();
  return {
      values[kSigmaOffset + 0],
      values[kSigmaOffset + 1],
      values[kSigmaOffset + 2],
  };
}

adapters::Vector3
ReadPredictionError(const ::drake::systems::Context<double> &context,
                    int discrete_state_index) {
  const auto &values = context.get_discrete_state(discrete_state_index).value();
  return {
      values[kPredictionErrorOffset + 0],
      values[kPredictionErrorOffset + 1],
      values[kPredictionErrorOffset + 2],
  };
}

adapters::Vector3 ComputeBound(const adapters::Vector3 &sigma_hat,
                               double disturbance_margin_mps2,
                               double disturbance_coupling, double sign) {
  adapters::Vector3 bound{};
  for (int axis = 0; axis < 3; ++axis) {
    bound[axis] = sign * (disturbance_margin_mps2 +
                          disturbance_coupling * std::abs(sigma_hat[axis]));
  }
  return bound;
}

} // namespace

L1GpacDrakeWrapper::L1GpacDrakeWrapper(
    const adapters::GpacSeamAdapterParams &params,
    double nominal_update_period_seconds)
    : params_(params), update_period_seconds_(nominal_update_period_seconds) {
  position_port_ = this->DeclareVectorInputPort("position", 3).get_index();
  velocity_port_ = this->DeclareVectorInputPort("velocity", 3).get_index();
  commanded_acceleration_port_ =
      this->DeclareVectorInputPort("commanded_acceleration", 3).get_index();

  controller_disturbance_port_ =
      this->DeclareVectorOutputPort(
              "controller_disturbance", 3,
              &L1GpacDrakeWrapper::CalcControllerDisturbance)
          .get_index();
  filtered_compensation_port_ =
      this->DeclareVectorOutputPort(
              "filtered_compensation", 3,
              &L1GpacDrakeWrapper::CalcFilteredCompensation)
          .get_index();
  safety_lower_bound_port_ =
      this->DeclareVectorOutputPort("safety_lower_bound", 3,
                                    &L1GpacDrakeWrapper::CalcSafetyLowerBound)
          .get_index();
  safety_upper_bound_port_ =
      this->DeclareVectorOutputPort("safety_upper_bound", 3,
                                    &L1GpacDrakeWrapper::CalcSafetyUpperBound)
          .get_index();
  raw_sigma_port_ =
      this->DeclareVectorOutputPort("raw_sigma_hat", 3,
                                    &L1GpacDrakeWrapper::CalcRawSigma)
          .get_index();
  prediction_error_port_ =
      this->DeclareVectorOutputPort("prediction_error", 3,
                                    &L1GpacDrakeWrapper::CalcPredictionError)
          .get_index();

  discrete_state_index_ = this->DeclareDiscreteState(kStateSize);
  this->DeclarePeriodicDiscreteUpdateEvent(update_period_seconds_, 0.0,
                                           &L1GpacDrakeWrapper::UpdateAdapter);
}

const ::drake::systems::InputPort<double> &
L1GpacDrakeWrapper::get_position_input_port() const {
  return this->get_input_port(position_port_);
}

const ::drake::systems::InputPort<double> &
L1GpacDrakeWrapper::get_velocity_input_port() const {
  return this->get_input_port(velocity_port_);
}

const ::drake::systems::InputPort<double> &
L1GpacDrakeWrapper::get_commanded_acceleration_input_port() const {
  return this->get_input_port(commanded_acceleration_port_);
}

const ::drake::systems::OutputPort<double> &
L1GpacDrakeWrapper::get_controller_disturbance_output_port() const {
  return this->get_output_port(controller_disturbance_port_);
}

const ::drake::systems::OutputPort<double> &
L1GpacDrakeWrapper::get_filtered_compensation_output_port() const {
  return this->get_output_port(filtered_compensation_port_);
}

const ::drake::systems::OutputPort<double> &
L1GpacDrakeWrapper::get_safety_lower_bound_output_port() const {
  return this->get_output_port(safety_lower_bound_port_);
}

const ::drake::systems::OutputPort<double> &
L1GpacDrakeWrapper::get_safety_upper_bound_output_port() const {
  return this->get_output_port(safety_upper_bound_port_);
}

const ::drake::systems::OutputPort<double> &
L1GpacDrakeWrapper::get_raw_sigma_output_port() const {
  return this->get_output_port(raw_sigma_port_);
}

const ::drake::systems::OutputPort<double> &
L1GpacDrakeWrapper::get_prediction_error_output_port() const {
  return this->get_output_port(prediction_error_port_);
}

::drake::systems::EventStatus L1GpacDrakeWrapper::UpdateAdapter(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::DiscreteValues<double> *discrete_state) const {
  const auto position = ReadVector3(context, get_position_input_port());
  const auto velocity = ReadVector3(context, get_velocity_input_port());
  const auto commanded_acceleration =
      ReadVector3(context, get_commanded_acceleration_input_port());

  ::tether_grace::gpac_fault_tolerance::L1AdaptiveCore core(params_.l1_params);
  core.set_state(ReadStoredState(context, discrete_state_index_));
  const auto &updated_state = core.Update(
      position, velocity, commanded_acceleration, update_period_seconds_);

  auto &state_vector =
      discrete_state->get_mutable_vector(discrete_state_index_);
  WriteStoredState(updated_state, &state_vector);
  return ::drake::systems::EventStatus::Succeeded();
}

void L1GpacDrakeWrapper::SetDefaultState(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::State<double> *state) const {
  const auto position = ReadVector3(context, get_position_input_port());
  const auto velocity = ReadVector3(context, get_velocity_input_port());

  ::tether_grace::gpac_fault_tolerance::L1AdaptiveState initial_state{};
  initial_state.predicted_position = position;
  initial_state.predicted_velocity = velocity;

  auto &state_vector = state->get_mutable_discrete_state(discrete_state_index_);
  WriteStoredState(initial_state, &state_vector);
}

void L1GpacDrakeWrapper::CalcControllerDisturbance(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  WriteVector3(ReadCompensation(context, discrete_state_index_), output);
}

void L1GpacDrakeWrapper::CalcFilteredCompensation(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  WriteVector3(ReadCompensation(context, discrete_state_index_), output);
}

void L1GpacDrakeWrapper::CalcSafetyLowerBound(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  WriteVector3(ComputeBound(ReadSigmaHat(context, discrete_state_index_),
                            params_.disturbance_margin_mps2,
                            params_.disturbance_coupling, -1.0),
               output);
}

void L1GpacDrakeWrapper::CalcSafetyUpperBound(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  WriteVector3(ComputeBound(ReadSigmaHat(context, discrete_state_index_),
                            params_.disturbance_margin_mps2,
                            params_.disturbance_coupling, 1.0),
               output);
}

void L1GpacDrakeWrapper::CalcRawSigma(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  WriteVector3(ReadSigmaHat(context, discrete_state_index_), output);
}

void L1GpacDrakeWrapper::CalcPredictionError(
    const ::drake::systems::Context<double> &context,
    ::drake::systems::BasicVector<double> *output) const {
  WriteVector3(ReadPredictionError(context, discrete_state_index_), output);
}

} // namespace tether_grace::gpac_fault_tolerance::drake
