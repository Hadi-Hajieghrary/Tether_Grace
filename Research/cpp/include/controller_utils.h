#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "quadcopter_controller.h"  // TrajectoryWaypoint

namespace quad_rope_lift {

/// Compute the payload reference pose/velocity at time t from a waypoint
/// list. Shared between `DecentralizedLocalController` and
/// `MpcLocalController` so both use the SAME interpolation logic.
inline void ComputePayloadReferenceAt(
    double t, const std::vector<TrajectoryWaypoint>& wps,
    Eigen::Vector3d* p_ref, Eigen::Vector3d* v_ref) {
  double seg_start = 0.0;
  for (size_t i = 0; i < wps.size(); ++i) {
    const auto& wp = wps[i];
    const double seg_end = wp.arrival_time;
    const double hold_end = seg_end + wp.hold_time;
    if (t <= seg_end) {
      if (i == 0) {
        *p_ref = wp.position;
        v_ref->setZero();
      } else {
        const auto& pv = wps[i - 1];
        const double dur = seg_end - seg_start;
        if (dur > 1e-6) {
          const double alpha = (t - seg_start) / dur;
          *p_ref = (1.0 - alpha) * pv.position + alpha * wp.position;
          *v_ref = (wp.position - pv.position) / dur;
        } else {
          *p_ref = wp.position;
          v_ref->setZero();
        }
      }
      return;
    } else if (t <= hold_end) {
      *p_ref = wp.position;
      v_ref->setZero();
      return;
    }
    seg_start = hold_end;
  }
  *p_ref = wps.back().position;
  v_ref->setZero();
}

/// Compute the drone formation-slot reference from the payload reference.
inline void ComputeSlotReferenceAt(
    double t, const std::vector<TrajectoryWaypoint>& wps,
    const Eigen::Vector3d& formation_offset, double rope_drop,
    Eigen::Vector3d* p_slot, Eigen::Vector3d* v_slot) {
  Eigen::Vector3d p_L_ref, v_L_ref;
  ComputePayloadReferenceAt(t, wps, &p_L_ref, &v_L_ref);
  *p_slot = p_L_ref + formation_offset;
  p_slot->z() += rope_drop;
  *v_slot = v_L_ref;
}

/// Solve the discrete algebraic Riccati equation (DARE)
///     P = Qs + As' P As - As' P Bs (Rs + Bs' P Bs)^{-1} Bs' P As
/// for a 2-state scalar-input scalar-axis double integrator. Iterated
/// fixed-point (theory-doc Eq. 40). Converges in ~30 iterations to
/// machine precision when (As, Bs) is stabilisable and (Qs^{1/2}, As)
/// is detectable. Returns the unique PD solution Ps.
inline Eigen::Matrix2d SolveScalarDARE(const Eigen::Matrix2d& As,
                                       const Eigen::Vector2d& Bs,
                                       const Eigen::Matrix2d& Qs,
                                       double Rs,
                                       int max_iter = 200,
                                       double tol = 1e-12) {
  Eigen::Matrix2d P = Qs;
  for (int k = 0; k < max_iter; ++k) {
    const Eigen::Matrix2d AtP = As.transpose() * P;
    const Eigen::Vector2d BtP = Bs.transpose() * P;
    const double denom = Rs + BtP.dot(Bs);
    const Eigen::Matrix2d P_new = Qs + AtP * As
        - (AtP * Bs) * (BtP.transpose() * As) / denom;
    const double diff = (P_new - P).cwiseAbs().maxCoeff();
    P = P_new;
    if (diff < tol) break;
  }
  return P;
}

/// LQR gain for the 2-state scalar-input DARE.
inline Eigen::RowVector2d DARELqrGain(const Eigen::Matrix2d& As,
                                       const Eigen::Vector2d& Bs,
                                       const Eigen::Matrix2d& Ps,
                                       double Rs) {
  const double denom = Rs + Bs.dot(Ps * Bs);
  return (Bs.transpose() * Ps * As) / denom;
}

/// Closed-loop spectral radius of As − Bs·Ks for the 2x2 double-
/// integrator DARE solution. Use for ctor-time sanity check.
inline double DARESpectralRadius(const Eigen::Matrix2d& As,
                                  const Eigen::Vector2d& Bs,
                                  const Eigen::RowVector2d& Ks) {
  const Eigen::Matrix2d Acl = As - Bs * Ks;
  Eigen::EigenSolver<Eigen::Matrix2d> es(Acl);
  const auto eigs = es.eigenvalues();
  return std::max(std::abs(eigs[0]), std::abs(eigs[1]));
}

}  // namespace quad_rope_lift
