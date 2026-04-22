#pragma once

#include <deque>
#include <random>

#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Models the one-broadcast fault-id channel between `FaultDetector`
/// and `FormationCoordinator`. Introduces a fixed end-to-end delay and
/// an independent Bernoulli drop per emission. When a packet is
/// dropped, the output holds its previous value (no automatic
/// retransmission; this is the worst-case deployment model).
class CommChannelModel final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CommChannelModel);

  struct Params {
    double delay_seconds   = 0.0;
    double drop_rate       = 0.0;    // ∈ [0, 1]
    double update_period   = 5e-3;
    int    seed            = 11;
  };

  explicit CommChannelModel(const Params& params)
      : params_(params), rng_(params.seed) {
    in_port_ = DeclareVectorInputPort("fault_id_in",
                   drake::systems::BasicVector<double>(1)).get_index();
    out_port_ = DeclareVectorOutputPort(
        "fault_id_out", drake::systems::BasicVector<double>(1),
        &CommChannelModel::CalcOutput).get_index();
    Eigen::VectorXd init(1);
    init[0] = -1.0;
    latched_ = DeclareDiscreteState(init);
    DeclarePeriodicUnrestrictedUpdateEvent(
        params_.update_period, 0.0, &CommChannelModel::Tick);
  }

  const drake::systems::InputPort<double>& get_fault_id_input_port() const {
    return get_input_port(in_port_);
  }
  const drake::systems::OutputPort<double>& get_fault_id_output_port() const {
    return get_output_port(out_port_);
  }

 private:
  drake::systems::EventStatus Tick(
      const drake::systems::Context<double>& ctx,
      drake::systems::State<double>* state) const {
    const double fault_in = get_input_port(in_port_).Eval(ctx)[0];
    if (fault_in < 0.0) return drake::systems::EventStatus::Succeeded();
    const double release_time = ctx.get_time() + params_.delay_seconds;
    std::uniform_real_distribution<double> u(0.0, 1.0);
    if (u(rng_) < params_.drop_rate) {
      return drake::systems::EventStatus::Succeeded();
    }
    pending_.push_back({release_time, fault_in});
    // Release any packets whose time has come.
    while (!pending_.empty() && pending_.front().t <= ctx.get_time()) {
      auto& mut_vec = state->get_mutable_discrete_state(latched_);
      auto xd = mut_vec.get_mutable_value();
      xd[0] = pending_.front().id;
      pending_.pop_front();
    }
    return drake::systems::EventStatus::Succeeded();
  }

  void CalcOutput(const drake::systems::Context<double>& ctx,
                  drake::systems::BasicVector<double>* out) const {
    const auto& xd = ctx.get_discrete_state(latched_).value();
    out->SetAtIndex(0, xd[0]);
  }

  struct Packet { double t; double id; };
  Params params_;
  int in_port_{-1};
  int out_port_{-1};
  drake::systems::DiscreteStateIndex latched_;
  mutable std::mt19937_64 rng_;
  mutable std::deque<Packet> pending_;
};

}  // namespace quad_rope_lift
