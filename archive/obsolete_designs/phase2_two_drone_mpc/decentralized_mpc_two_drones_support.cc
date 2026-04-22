#include "decentralized_mpc_two_drones.h"

namespace quad_rope_lift {

CableSeveranceInjector::CableSeveranceInjector(int drone_index,
                                              double severance_time)
    : drone_index_(drone_index), severance_time_(severance_time) {
  // Input port: cable tension from measurement
  DeclareInputPort("T_cable_in", drake::systems::kVectorValued, 1);

  // Output port: cable tension with fault injection
  DeclareVectorOutputPort("T_cable_out", drake::systems::BasicVector<double>(1),
                          &CableSeveranceInjector::CalcTensionWithFault);
}

void CableSeveranceInjector::CalcTensionWithFault(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  // Read input tension
  auto T_in = get_input_port(0)
      .Eval<drake::systems::BasicVector<double>>(context)
      .value();
  double T = T_in(0);

  // Inject fault if time >= severance_time
  if (severance_time_ >= 0.0 && context.get_time() >= severance_time_) {
    T = 0.0;  // Cable severed: tension drops to zero
  }

  output->SetFromVector(Eigen::VectorXd::Ones(1) * T);
}

}  // namespace quad_rope_lift
