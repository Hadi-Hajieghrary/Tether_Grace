#pragma once

#include <vector>

#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// A Drake `LeafSystem` that infers the faulted-drone index from the
/// N-vector of scalar rope tensions (per `theory_reshaping_extension.md`
/// §E). Drone `i` is declared faulted once its rope tension has stayed
/// below `tension_threshold` for `detect_duration` seconds continuously.
/// Latched: once a fault is detected, the output stays at that drone's
/// index until the simulation ends (idempotent — no re-triggering).
///
/// Input port:
///   - `tensions` — N-vector of scalar rope tensions (typically wired
///     from the per-drone `ZeroOrderHold`/`Demultiplexer` top-segment
///     tension channel).
///
/// Output port:
///   - `fault_id` — scalar double: −1 if no fault has been latched,
///     otherwise the index of the latched drone (0 … N−1).
class FaultDetector final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FaultDetector);

  struct Params {
    int    num_drones        = 4;
    double tension_threshold = 0.5;   // N  (below this → candidate)
    double detect_duration   = 0.1;   // s  (sustained below threshold)
    /// Update period. Should match the sim rate or a convenient divisor.
    double update_period     = 2e-4;
  };

  explicit FaultDetector(const Params& params)
      : params_(params) {
    tensions_port_ =
        DeclareVectorInputPort("tensions",
                               drake::systems::BasicVector<double>(
                                   params_.num_drones))
            .get_index();
    fault_id_port_ = DeclareVectorOutputPort(
        "fault_id", drake::systems::BasicVector<double>(1),
        &FaultDetector::CalcFaultId).get_index();

    // Discrete state layout (N + 1 doubles):
    //   [0 .. N-1]  below_thresh_since_i  (−1 if not-yet below)
    //   [N]         latched_fault_id (−1 if none)
    Eigen::VectorXd init =
        -Eigen::VectorXd::Ones(params_.num_drones + 1);
    state_index_ = DeclareDiscreteState(init);

    DeclarePeriodicUnrestrictedUpdateEvent(
        params_.update_period, 0.0, &FaultDetector::UpdateDetector);
  }

  const drake::systems::InputPort<double>& get_tensions_input_port() const {
    return get_input_port(tensions_port_);
  }
  const drake::systems::OutputPort<double>& get_fault_id_output_port() const {
    return get_output_port(fault_id_port_);
  }

 private:
  drake::systems::EventStatus UpdateDetector(
      const drake::systems::Context<double>& ctx,
      drake::systems::State<double>* state) const {
    const double t = ctx.get_time();
    const auto& T = get_input_port(tensions_port_).Eval(ctx);
    auto& mut_vec = state->get_mutable_discrete_state(state_index_);
    auto xd = mut_vec.get_mutable_value();
    // Idempotent: once latched, never re-trigger.
    if (xd[params_.num_drones] >= 0.0) {
      return drake::systems::EventStatus::Succeeded();
    }
    for (int i = 0; i < params_.num_drones; ++i) {
      if (T(i) < params_.tension_threshold) {
        if (xd[i] < 0.0) xd[i] = t;
        if (t - xd[i] >= params_.detect_duration) {
          xd[params_.num_drones] = static_cast<double>(i);
          return drake::systems::EventStatus::Succeeded();
        }
      } else {
        xd[i] = -1.0;  // reset hysteresis on this drone
      }
    }
    return drake::systems::EventStatus::Succeeded();
  }

  void CalcFaultId(const drake::systems::Context<double>& ctx,
                   drake::systems::BasicVector<double>* out) const {
    const auto& xd =
        ctx.get_discrete_state(state_index_).value();
    out->SetAtIndex(0, xd[params_.num_drones]);
  }

  Params params_;
  int tensions_port_{-1};
  int fault_id_port_{-1};
  drake::systems::DiscreteStateIndex state_index_;
};

}  // namespace quad_rope_lift
