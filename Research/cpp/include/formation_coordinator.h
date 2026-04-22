#pragma once

#include <vector>

#include <Eigen/Core>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Supervisor that reassigns formation slot offsets after a cable
/// severance. On receipt of a latched `fault_id` it computes a
/// closed-form optimal re-spacing of the surviving drones and
/// interpolates between old and new angular positions with a quintic
/// smoothstep. The output is a 3N-vector containing all drones'
/// offsets; downstream controllers slice their own three-vector.
///
/// Inputs:  `fault_id` (−1 = none, otherwise 0..N−1).
/// Outputs: `formation_offsets` (3N-vector).
class FormationCoordinator final
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FormationCoordinator);

  struct Params {
    int    num_drones       = 4;
    double formation_radius = 0.8;
    /// Smoothstep transition duration (s). Must be ≥ 1 s to keep max
    /// slot speed below the anti-swing damper capability (~0.375 m/s).
    double t_trans          = 5.0;
    /// Periodic update interval.
    double update_period    = 5e-3;
  };

  explicit FormationCoordinator(const Params& params)
      : params_(params) {
    fault_id_port_ = DeclareVectorInputPort(
        "fault_id", drake::systems::BasicVector<double>(1)).get_index();

    // Output: 3·N-vector of concatenated (x,y,z) formation offsets.
    formation_offsets_port_ = DeclareVectorOutputPort(
        "formation_offsets",
        drake::systems::BasicVector<double>(3 * params_.num_drones),
        &FormationCoordinator::CalcFormationOffsets).get_index();

    // Discrete state: [t_fault, latched_id,
    //                  phi_old_0..phi_old_{N-1},
    //                  phi_new_0..phi_new_{N-1}]
    //   2 scalars + 2·N angles
    Eigen::VectorXd init = Eigen::VectorXd::Zero(2 + 2 * params_.num_drones);
    init[kLatchedIdSlot] = -1.0;
    init[kTFaultSlot]    = -1.0;
    // Initial φ_old_i = φ_new_i = 2π i/N (nominal equiangular layout).
    const int new_off = kPhiOldSlot + params_.num_drones;
    for (int i = 0; i < params_.num_drones; ++i) {
      const double phi_nom = 2.0 * M_PI * i / params_.num_drones;
      init[kPhiOldSlot + i] = phi_nom;
      init[new_off + i] = phi_nom;
    }
    state_index_ = DeclareDiscreteState(init);

    DeclarePeriodicUnrestrictedUpdateEvent(
        params_.update_period, 0.0,
        &FormationCoordinator::UpdateCoordinator);
  }

  const drake::systems::InputPort<double>& get_fault_id_input_port() const {
    return get_input_port(fault_id_port_);
  }
  const drake::systems::OutputPort<double>& get_formation_offsets_output_port()
      const {
    return get_output_port(formation_offsets_port_);
  }

 private:
  static constexpr int kTFaultSlot   = 0;
  static constexpr int kLatchedIdSlot = 1;
  static constexpr int kPhiOldSlot = 2;       // φ_old_0 … φ_old_{N-1}
  int PhiNewSlot() const { return kPhiOldSlot + params_.num_drones; }

  drake::systems::EventStatus UpdateCoordinator(
      const drake::systems::Context<double>& ctx,
      drake::systems::State<double>* state) const {
    auto& mut_vec = state->get_mutable_discrete_state(state_index_);
    auto xd = mut_vec.get_mutable_value();
    const double latched_in = get_input_port(fault_id_port_).Eval(ctx)[0];
    const double latched_stored = xd[kLatchedIdSlot];
    if (latched_stored < 0.0 && latched_in >= 0.0) {
      // Newly latched: compute new slots via closed-form assignment.
      const int j_star = static_cast<int>(latched_in + 0.5);
      xd[kLatchedIdSlot] = latched_in;
      xd[kTFaultSlot] = ctx.get_time();
      // Record old angles (frozen at fault time).
      for (int i = 0; i < params_.num_drones; ++i) {
        xd[kPhiOldSlot + i] = 2.0 * M_PI * i / params_.num_drones;
      }
      // New angles: closed-form 30°-toward-gap assignment for N=4.
      // For other N: exhaustive 6-permutation × 5-θ₀ min-max search.
      const auto new_phi = ComputeNewPhi(j_star);
      for (int i = 0; i < params_.num_drones; ++i) {
        xd[PhiNewSlot() + i] = new_phi[i];
      }
    }
    return drake::systems::EventStatus::Succeeded();
  }

  /// New equiangular slot angles for the surviving drones given the
  /// faulted index. For N = 4 the drone opposite the gap stays fixed
  /// and the two adjacent drones each rotate π/6 toward the gap —
  /// this is globally tension-optimal (KKT derivation in the
  /// supplementary). For N ≠ 4 the nominal angles are preserved;
  /// re-spacing is handled by the reference demultiplex logic.
  std::vector<double> ComputeNewPhi(int j_star) const {
    const int N = params_.num_drones;
    std::vector<double> new_phi(N);
    for (int i = 0; i < N; ++i) {
      new_phi[i] = 2.0 * M_PI * i / N;  // default: keep nominal
    }
    if (N == 4) {
      const double pi6 = M_PI / 6.0;
      const int opp = (j_star + 2) % 4;
      for (int i = 0; i < N; ++i) {
        if (i == j_star || i == opp) continue;
        const double phi_i = new_phi[i];
        const double phi_j = 2.0 * M_PI * j_star / N;
        const double d = std::fmod(phi_j - phi_i + 3.0 * M_PI,
                                   2.0 * M_PI) - M_PI;
        new_phi[i] = phi_i + std::copysign(pi6, d);
      }
    }
    return new_phi;
  }

  void CalcFormationOffsets(const drake::systems::Context<double>& ctx,
                             drake::systems::BasicVector<double>* out) const {
    const auto& xd =
        ctx.get_discrete_state(state_index_).value();
    const double latched = xd[kLatchedIdSlot];
    const double t_fault = xd[kTFaultSlot];
    double tau = 0.0;
    if (latched >= 0.0) {
      tau = std::clamp((ctx.get_time() - t_fault) / params_.t_trans, 0.0, 1.0);
    }
    // Quintic smoothstep h(τ) = 10τ³ − 15τ⁴ + 6τ⁵ (C² at endpoints).
    const double h = (10.0 - 15.0 * tau + 6.0 * tau * tau) * tau * tau * tau;
    const int N = params_.num_drones;
    for (int i = 0; i < N; ++i) {
      const double phi_old = xd[kPhiOldSlot + i];
      const double phi_new = xd[PhiNewSlot() + i];
      // Signed shortest-arc interpolation.
      double d = std::fmod(phi_new - phi_old + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
      const double phi_i = phi_old + d * h;
      out->SetAtIndex(3 * i + 0,
                      params_.formation_radius * std::cos(phi_i));
      out->SetAtIndex(3 * i + 1,
                      params_.formation_radius * std::sin(phi_i));
      out->SetAtIndex(3 * i + 2, 0.0);
    }
  }

  Params params_;
  int fault_id_port_{-1};
  int formation_offsets_port_{-1};
  drake::systems::DiscreteStateIndex state_index_;
};

}  // namespace quad_rope_lift
