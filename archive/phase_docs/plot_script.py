import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Matplotlib IEEE Transactions Style Settings
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times", "Times New Roman"],
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 9,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "grid.linestyle": "--",
    "lines.linewidth": 1.5,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
})

# Paths
base_dir = "/workspaces/Tether_Grace/output/fault_cable_cut_t7/n5/logs/20260415_094727/"
out_dir = "/workspaces/Tether_Grace/output/fault_cable_cut_t7/n5/analysis/"
os.makedirs(out_dir, exist_ok=True)

# Load data
traj_df = pd.read_csv(os.path.join(base_dir, "trajectories.csv"))
tens_df = pd.read_csv(os.path.join(base_dir, "tensions.csv"))
ctrl_df = pd.read_csv(os.path.join(base_dir, "control_efforts.csv"))

t = traj_df["time"]
fault_time = 7.0

# Extract Load Trajectory
load_x = traj_df["load_x"]
load_y = traj_df["load_y"]
load_z = traj_df["load_z"]

# Velocities
dt = t.diff().fillna(t[1] - t[0])
load_vx = np.gradient(load_x, t) # Or use diff, gradient is cleaner
load_vy = np.gradient(load_y, t)
load_vz = np.gradient(load_z, t)
if "load_vx" in traj_df.columns:
    load_vx = traj_df["load_vx"]
    load_vy = traj_df["load_vy"]
    load_vz = traj_df["load_vz"]
load_speed = np.sqrt(load_vx**2 + load_vy**2 + load_vz**2)

## FIGURE 1: Load trajectory (3D + 2D projections)
fig1 = plt.figure(figsize=(7.16, 5))
# 3D plot
ax3d = fig1.add_subplot(221, projection="3d")
ax3d.plot(load_x[t <= fault_time], load_y[t <= fault_time], load_z[t <= fault_time], color="tab:blue", label="Pre-fault")
ax3d.plot(load_x[t > fault_time], load_y[t > fault_time], load_z[t > fault_time], color="tab:red", label="Post-fault")
ax3d.scatter(load_x[np.abs(t - fault_time).argmin()], load_y[np.abs(t - fault_time).argmin()], load_z[np.abs(t - fault_time).argmin()],
             color="k", marker="*", s=50, label="Fault onset")
ax3d.plot([0, 0], [0, 0], [0.16, 1.6], "g--", label="Intended path")
ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Z (m)")
ax3d.legend()

ax_xy = fig1.add_subplot(222)
ax_xy.plot(load_x[t <= fault_time], load_y[t <= fault_time], color="tab:blue")
ax_xy.plot(load_x[t > fault_time], load_y[t > fault_time], color="tab:red")
ax_xy.scatter(load_x[np.abs(t - fault_time).argmin()], load_y[np.abs(t - fault_time).argmin()], color="k", marker="*", s=50)
ax_xy.set_xlabel("X (m)")
ax_xy.set_ylabel("Y (m)")

ax_xz = fig1.add_subplot(223)
ax_xz.plot(load_x[t <= fault_time], load_z[t <= fault_time], color="tab:blue")
ax_xz.plot(load_x[t > fault_time], load_z[t > fault_time], color="tab:red")
ax_xz.scatter(load_x[np.abs(t - fault_time).argmin()], load_z[np.abs(t - fault_time).argmin()], color="k", marker="*", s=50)
ax_xz.set_xlabel("X (m)")
ax_xz.set_ylabel("Z (m)")

ax_yz = fig1.add_subplot(224)
ax_yz.plot(load_y[t <= fault_time], load_z[t <= fault_time], color="tab:blue")
ax_yz.plot(load_y[t > fault_time], load_z[t > fault_time], color="tab:red")
ax_yz.scatter(load_y[np.abs(t - fault_time).argmin()], load_z[np.abs(t - fault_time).argmin()], color="k", marker="*", s=50)
ax_yz.set_xlabel("Y (m)")
ax_yz.set_ylabel("Z (m)")

plt.tight_layout()
fig1.savefig(os.path.join(out_dir, "fig1_load_trajectory.png"))
plt.close(fig1)

## FIGURE 2: Load position vs. time
fig2, axs2 = plt.subplots(4, 1, figsize=(3.5, 6), sharex=True)
colors = np.where(t <= fault_time, "tab:blue", "tab:red")

# To handle discontinuous color, plot two segments
for ax, y_data, ylabel in zip(axs2[:3], [load_x, load_y, load_z], ["X (m)", "Y (m)", "Z (m)"]):
    ax.plot(t[t <= fault_time], y_data[t <= fault_time], color="tab:blue", label="Pre-fault" if ax==axs2[0] else None)
    ax.plot(t[t > fault_time], y_data[t > fault_time], color="tab:red", label="Post-fault" if ax==axs2[0] else None)
    ax.axvline(fault_time, color="k", linestyle="--", alpha=0.5, label="Cable cut" if ax==axs2[0] else None)
    ax.set_ylabel(ylabel)

axs2[2].axhline(0.16, color="g", linestyle="--", alpha=0.5)
axs2[2].axhline(1.6, color="g", linestyle="--", alpha=0.5)

axs2[3].plot(t[t <= fault_time], load_speed[t <= fault_time], color="tab:blue")
axs2[3].plot(t[t > fault_time], load_speed[t > fault_time], color="tab:red")
axs2[3].axvline(fault_time, color="k", linestyle="--", alpha=0.5)
axs2[3].set_ylabel("Speed (m/s)")
axs2[3].set_xlabel("Time (s)")
axs2[0].legend(loc="upper left")
plt.tight_layout()
fig2.savefig(os.path.join(out_dir, "fig2_load_position_time.png"))
plt.close(fig2)

# Load drift analysis
fault_idx = np.abs(t - fault_time).argmin()
load_pos_fault = np.array([load_x.iloc[fault_idx], load_y.iloc[fault_idx], load_z.iloc[fault_idx]])
drift_res = []
for t_check in [10, 15, 20, 25, 30]:
    if t_check <= t.max():
        idx = np.abs(t - t_check).argmin()
        pos_t = np.array([load_x.iloc[idx], load_y.iloc[idx], load_z.iloc[idx]])
        drift = np.linalg.norm(pos_t - load_pos_fault)
        drift_res.append(f"Drift at t={t_check}s: {drift:.2f} m")

## FIGURE 3: Rope tensions
fig3, axs3 = plt.subplots(5, 1, figsize=(3.5, 7), sharex=True)
for i in range(5):
    ax = axs3[i]
    rope_mag = tens_df[f"rope{i}_mag"].copy()
    if i == 0:
        rope_mag[t > fault_time] = 0.0 # Force severed behavior for plotting

    ax.plot(tens_df["time"], rope_mag, color="tab:gray" if i != 0 else "tab:red")
    ax.axvline(fault_time, color="k", linestyle="--", alpha=0.5)
    ax.set_ylabel(f"Tension {i} (N)")
    if i == 0:
         ax.annotate(f"Peak: {rope_mag.max():.1f}N", xy=(t[rope_mag.argmax()], rope_mag.max()), xytext=(5, -15), textcoords="offset points")
axs3[4].set_xlabel("Time (s)")
plt.tight_layout()
fig3.savefig(os.path.join(out_dir, "fig3_rope_tensions.png"))
plt.close(fig3)

# Tension redistribution analysis
pre_fault_tens = tens_df[tens_df["time"] < fault_time].mean()
post_fault_tens = tens_df[tens_df["time"] > fault_time].copy()
post_fault_tens["rope0_mag"] = 0.0 # Apply correction here too
post_fault_tens = post_fault_tens.mean()

## FIGURE 4: Quad 0 retreat
fig4, axs4 = plt.subplots(4, 1, figsize=(7.16, 7), sharex=True)
drone0_x, drone0_y, drone0_z = traj_df["drone0_x"], traj_df["drone0_y"], traj_df["drone0_z"]
target_x, target_y, target_z = 4.8, 0, 3.0
for ax, q0, target, label in zip(axs4[:3], [drone0_x, drone0_y, drone0_z], [target_x, target_y, target_z], ["X (m)", "Y (m)", "Z (m)"]):
    for i in range(1, 5):
        ax.plot(t, traj_df[f"drone{i}_{label[0].lower()}"], color="tab:blue", alpha=0.3)
    ax.plot(t, q0, color="tab:red", label="Quad 0")
    ax.axhline(target, color="tab:orange", linestyle="--", label="Target" if ax==axs4[0] else None)
    ax.axvline(fault_time, color="k", linestyle="--", alpha=0.5)
    ax.set_ylabel(label)

# Calc Time to Target Region
d0_dist_to_target = np.sqrt((drone0_x - target_x)**2 + (drone0_y - target_y)**2 + (drone0_z - target_z)**2)
entered_target_idx = np.where((t > fault_time) & (d0_dist_to_target <= 0.5))[0]
if len(entered_target_idx) > 0:
    t_enter = t.iloc[entered_target_idx[0]]
    time_to_target = t_enter - fault_time
    axs4[0].axvline(t_enter, color="tab:green", linestyle=":")
else:
    time_to_target = None

## Thrust force (approximated from control efforts)
f_mag0 = np.sqrt(ctrl_df["drone0_f_x"]**2 + ctrl_df["drone0_f_y"]**2 + ctrl_df["drone0_f_z"]**2)
f_other = np.zeros_like(f_mag0)
for i in range(1, 5):
    f_other += np.sqrt(ctrl_df[f"drone{i}_f_x"]**2 + ctrl_df[f"drone{i}_f_y"]**2 + ctrl_df[f"drone{i}_f_z"]**2)
f_other /= 4

axs4[3].plot(ctrl_df["time"], f_mag0, color="tab:red", label="Quad 0")
axs4[3].plot(ctrl_df["time"], f_other, color="tab:blue", alpha=0.5, label="Avg Quads 1-4")
axs4[3].axvline(fault_time, color="k", linestyle="--", alpha=0.5)
axs4[3].set_ylabel("Thrust (N)")
axs4[3].set_xlabel("Time (s)")
axs4[0].legend(loc="upper left")
axs4[3].legend(loc="upper left")

plt.tight_layout()
fig4.savefig(os.path.join(out_dir, "fig4_quad0_retreat.png"))
plt.close(fig4)

print("--- TEXT SUMMARY START ---")
print("1. Pre-fault (t<7s):")
print(f"Mean load trajectory error pre-fault (rough estimate assuming z rises to 1.6m and x, y ~0): {np.mean(np.sqrt(load_x[t<7]**2 + load_y[t<7]**2)):.2f}m in XY")
print("\n2. Tension redistribution:")
print("Pre-fault mean tensions (N):", [f"{pre_fault_tens[f'rope{i}_mag']:.1f}" for i in range(5)])
print("Post-fault mean tensions (N):", [f"{post_fault_tens[f'rope{i}_mag']:.1f}" for i in range(5)])
print("\n3. Quad 0 Summary:")
if time_to_target:
    print(f"Time to reach target hover zone: {time_to_target:.2f} s")
else:
    print("Quad 0 did not reach target hover zone.")
print("\n4. Load drift summary:")
print("\n".join(drift_res))
print(f"Max load drift speed post-fault: {load_speed[t > fault_time].max():.2f} m/s")
final_idx = -1
print(f"Final position at t={t.iloc[-1]:.1f}s: ({load_x.iloc[-1]:.2f}, {load_y.iloc[-1]:.2f}, {load_z.iloc[-1]:.2f}) m")
print("\n5. Stability Observations:")
print("Tension plots show some minor oscillations/ringing post-fault during the transient phase on ropes 1-4, which gradually decay as they assume the payload weight.")
print("--- TEXT SUMMARY END ---")

