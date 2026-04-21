#pragma once

#include <vector>

#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// A Drake LeafSystem that passes rope external forces through unchanged
/// until `fault_time` seconds, then outputs an empty force vector
/// (simulating instantaneous cable severance).
///
/// Wire it between RopeForceSystem::get_forces_output_port() and the
/// ExternallyAppliedSpatialForceMultiplexer input for the faulted quadcopter:
///
///   RopeForceSystem  ->  CableFaultGate  ->  ForceMultiplexer
///
class CableFaultGate final : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CableFaultGate);

  /// @param fault_time  Simulation time [s] at which the cable is cut.
  ///                    For t >= fault_time the output force vector is empty.
  explicit CableFaultGate(double fault_time) : fault_time_(fault_time) {
    using drake::Value;
    using drake::multibody::ExternallyAppliedSpatialForce;

    forces_input_port_ =
        DeclareAbstractInputPort(
            "forces_in",
            Value<std::vector<ExternallyAppliedSpatialForce<double>>>())
            .get_index();

    forces_output_port_ = DeclareAbstractOutputPort(
                              "forces_out", &CableFaultGate::CalcGatedForces)
                              .get_index();
  }

  const drake::systems::InputPort<double> &get_forces_input_port() const {
    return get_input_port(forces_input_port_);
  }

  const drake::systems::OutputPort<double> &get_forces_output_port() const {
    return get_output_port(forces_output_port_);
  }

private:
  void CalcGatedForces(
      const drake::systems::Context<double> &context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>
          *output) const {
    if (context.get_time() >= fault_time_) {
      // Cable severed — zero rope contribution to the plant.
      output->clear();
    } else {
      *output =
          get_input_port(forces_input_port_)
              .Eval<std::vector<
                  drake::multibody::ExternallyAppliedSpatialForce<double>>>(
                  context);
    }
  }

  double fault_time_;
  int forces_input_port_{-1};
  int forces_output_port_{-1};
};

/// A Drake LeafSystem that passes a 4D tension vector through unchanged until
/// `fault_time`, then outputs zeros (simulating cable severance in telemetry).
///
/// Wire it between RopeForceSystem::get_tension_output_port() and the
/// ZeroOrderHold tension input for the faulted quadcopter.
class TensionFaultGate final : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TensionFaultGate);

  explicit TensionFaultGate(double fault_time) : fault_time_(fault_time) {
    tension_input_port_ =
        DeclareVectorInputPort("tension_in", 4).get_index();
    tension_output_port_ =
        DeclareVectorOutputPort("tension_out", 4,
                                &TensionFaultGate::CalcGatedTension)
            .get_index();
  }

  const drake::systems::InputPort<double>& get_tension_input_port() const {
    return get_input_port(tension_input_port_);
  }
  const drake::systems::OutputPort<double>& get_tension_output_port() const {
    return get_output_port(tension_output_port_);
  }

private:
  void CalcGatedTension(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* output) const {
    if (context.get_time() >= fault_time_) {
      output->SetZero();
    } else {
      output->SetFromVector(
          get_input_port(tension_input_port_).Eval(context));
    }
  }

  double fault_time_;
  int tension_input_port_{-1};
  int tension_output_port_{-1};
};

} // namespace quad_rope_lift
