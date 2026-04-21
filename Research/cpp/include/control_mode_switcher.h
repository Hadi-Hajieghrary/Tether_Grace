#pragma once
///
/// @file control_mode_switcher.h
///
/// Drake LeafSystem that routes one of two ExternallyAppliedSpatialForce
/// abstract outputs based on simulation time.
///
///   t < fault_time  →  routes "normal" input (regular formation controller)
///   t >= fault_time →  routes "safe"   input (SafeHoverController)

#include <vector>

#include <drake/common/value.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/systems/framework/leaf_system.h>

namespace quad_rope_lift {

/// Selects between two ExternallyAppliedSpatialForce control outputs based on
/// the simulation clock.  Before @p fault_time port 0 ("normal") is forwarded;
/// at and after @p fault_time port 1 ("safe_hover") is forwarded.
class ControlModeSwitcher final : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ControlModeSwitcher);

  using ForceVec =
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>;

  explicit ControlModeSwitcher(double fault_time) : fault_time_(fault_time) {
    normal_port_ =
        DeclareAbstractInputPort("normal_ctrl", drake::Value<ForceVec>())
            .get_index();
    safe_port_ = DeclareAbstractInputPort("safe_ctrl", drake::Value<ForceVec>())
                     .get_index();
    output_port_ = DeclareAbstractOutputPort("active_ctrl",
                                             &ControlModeSwitcher::CalcOutput)
                       .get_index();
  }

  const drake::systems::InputPort<double> &get_normal_input_port() const {
    return get_input_port(normal_port_);
  }
  const drake::systems::InputPort<double> &get_safe_input_port() const {
    return get_input_port(safe_port_);
  }
  /// Returns the single output port carrying the active controller's forces.
  const drake::systems::OutputPort<double> &get_active_output_port() const {
    return get_output_port(output_port_);
  }

private:
  void CalcOutput(const drake::systems::Context<double> &ctx,
                  ForceVec *out) const {
    if (ctx.get_time() >= fault_time_) {
      *out = get_input_port(safe_port_).Eval<ForceVec>(ctx);
    } else {
      *out = get_input_port(normal_port_).Eval<ForceVec>(ctx);
    }
  }

  double fault_time_;
  int normal_port_{-1};
  int safe_port_{-1};
  int output_port_{-1};
};

} // namespace quad_rope_lift
