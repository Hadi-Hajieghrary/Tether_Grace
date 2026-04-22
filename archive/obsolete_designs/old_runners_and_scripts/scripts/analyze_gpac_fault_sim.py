#!/usr/bin/env python3
"""
analyze_gpac_fault_sim.py — Comprehensive IEEE-style post-processing for
the GPAC cable-fault simulation (N=4, lemniscate trajectory).

Generates 22 publication figures + numerical_summary.txt:

  Fig 01  3-D load trajectory (pre/post fault)
  Fig 02  Load X/Y/Z vs reference
  Fig 03  Tracking-error components + norms
  Fig 04  Rope tensions (fault rope zeroed after cut)
  Fig 05  Load altitude stability
  Fig 06  Fault-quad retreat trajectory
  Fig 07  All drone positions
  Fig 08  Thrust redistribution (Fz)
  Fig 09  Drone attitudes (roll/pitch/yaw)
  Fig 10  Geometric attitude errors eR
  Fig 11  Concurrent-learning mass estimates theta_hat
  Fig 12  ESO disturbance estimates
  Fig 13  Anti-swing cable forces
  Fig 14  CBF safety barrier values
  Fig 15  Cable direction vectors on S2
  Fig 16  ESKF drone position accuracy
  Fig 17  Load swing tilt angle
  Fig 18  Cable elevation angles
  Fig 19  Fault-transient zoom window
  Fig 20  Control torques
  Fig 21  XY overhead path
  Fig 22  Summary dashboard (6-panel)

Usage:
    python3 analyze_gpac_fault_sim.py \\
        --log-dir /workspaces/Tether_Grace/output/gpac/n4_cable_cut_t20/logs/YYYYMMDD_HHMMSS
"""
from __future__ import annotations
import argparse, math, os, sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd

# ── IEEE style ──────────────────────────────────────────────────────────────
_C = ["#1f77b4","#ff7f0e","#2ca02c","#d62728","#9467bd","#8c564b","#e377c2","#7f7f7f"]
_FAULT = "#d62728"
_REF   = "#555555"

def _ieee():
    plt.rcParams.update({
        "font.family":"serif","font.serif":["Times New Roman","DejaVu Serif"],
        "font.size":9,"axes.labelsize":9,"axes.titlesize":9,
        "legend.fontsize":7,"xtick.labelsize":8,"ytick.labelsize":8,
        "lines.linewidth":1.2,"axes.grid":True,"grid.alpha":0.35,
        "figure.dpi":150,"savefig.dpi":300,
        "savefig.bbox":"tight","savefig.pad_inches":0.025,
    })

def _vl(ax, tf, lbl=False):
    ax.axvline(tf, color=_FAULT, ls="--", lw=0.9,
               label=f"Cable cut  t={tf:.0f} s" if lbl else None)

def _save(fig, path):
    fig.savefig(path); plt.close(fig)
    print(f"  + {os.path.basename(path)}")

def _ds(q, fq):   # drone style
    return (_FAULT if q==fq else _C[q%len(_C)]), ("--" if q==fq else "-")

# ── Config / waypoints ──────────────────────────────────────────────────────
def _cfg(log_dir):
    cfg, p = {}, os.path.join(log_dir, "config.txt")
    if not os.path.exists(p): return cfg
    with open(p) as f:
        for raw in f:
            s = raw.strip()
            if not s or s.startswith("#"): continue
            if "=" in s:
                k,_,v = s.partition("="); cfg[k.strip()] = v.strip()
    return cfg

def _wps(cfg):
    try:
        oz = float(cfg.get("avg_rope_length",1.0))*1.15 + float(cfg.get("payload_radius",0.15)) + 0.1
    except Exception: oz = 0.0
    wps, i = [], 0
    while f"waypoint_{i}_arrival_time" in cfg:
        at  = float(cfg[f"waypoint_{i}_arrival_time"])
        ht  = float(cfg[f"waypoint_{i}_hold_time"])
        raw = cfg[f"waypoint_{i}_position"].strip("()").split(",")
        pos = np.array([float(v) for v in raw]); pos[2] -= oz
        wps.append((at, ht, pos)); i += 1
    return wps

def _ref_at(t, wps):
    if not wps: return np.zeros(3)
    if t <= wps[0][0]: return wps[0][2].copy()
    for i,(at,ht,pos) in enumerate(wps):
        he = at+ht
        if at<=t<=he: return pos.copy()
        if i+1<len(wps):
            an = wps[i+1][0]
            if he<t<=an:
                a = (t-he)/max(an-he,1e-9)
                return pos + a*(wps[i+1][2]-pos)
    return wps[-1][2].copy()

def _build_ref(t_arr, wps):
    if not wps: return None
    return np.vstack([_ref_at(t, wps) for t in t_arr])

# ── CSV helper ───────────────────────────────────────────────────────────────
def _csv(d, name, req=False):
    p = os.path.join(d, name)
    if not os.path.exists(p):
        if req: sys.exit(f"ERROR: required {name} not in {d}")
        print(f"  ! {name} missing"); return None
    df = pd.read_csv(p)
    print(f"  Loaded {name}: {len(df):,} rows × {len(df.columns)} cols")
    return df

# ── Figures ──────────────────────────────────────────────────────────────────

def fig01(df_t, tf, out):
    fig = plt.figure(figsize=(3.5,3.2)); ax = fig.add_subplot(111, projection="3d")
    t,x,y,z = (df_t[c].values for c in ["time","load_x","load_y","load_z"])
    pre,post = t<=tf, t>tf
    ax.plot(x[pre],y[pre],z[pre],color=_C[0],lw=1.2,label="Pre-fault")
    ax.plot(x[post],y[post],z[post],color=_FAULT,lw=1.2,label="Post-fault")
    if pre.any():
        ax.scatter(x[pre][-1],y[pre][-1],z[pre][-1],marker="x",s=50,color=_FAULT,zorder=5)
    ax.set_xlabel("X (m)",labelpad=2); ax.set_ylabel("Y (m)",labelpad=2); ax.set_zlabel("Z (m)",labelpad=2)
    ax.set_title("Load 3-D Trajectory"); ax.legend(fontsize=7); ax.tick_params(labelsize=7)
    plt.tight_layout(); _save(fig, os.path.join(out,"fig01_3d_trajectory.png"))

def fig02(df_t, ref, tf, out):
    fig,axs = plt.subplots(3,1,figsize=(5.5,5.0),sharex=True)
    t = df_t["time"].values
    for ax,col,ri,yl,c in zip(axs,["load_x","load_y","load_z"],[0,1,2],["X (m)","Y (m)","Z (m)"],_C):
        ax.plot(t, df_t[col].values, color=c, lw=1.2, label="Actual")
        if ref is not None: ax.plot(t, ref[:,ri], color=_REF, lw=0.9, ls="--", label="Reference")
        _vl(ax,tf,col=="load_x"); ax.set_ylabel(yl)
    axs[0].set_title("Load Position vs Time"); axs[0].legend(fontsize=7)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig02_load_position.png"))

def fig03(df_t, ref, tf, out):
    if ref is None: return
    t = df_t["time"].values
    ex = df_t["load_x"].values - ref[:,0]
    ey = df_t["load_y"].values - ref[:,1]
    ez = df_t["load_z"].values - ref[:,2]
    en = np.sqrt(ex**2+ey**2+ez**2)
    fig,axs = plt.subplots(4,1,figsize=(5.5,6.2),sharex=True)
    axs[0].plot(t,ex,color=_C[0],lw=1.0,label=r"$e_x$")
    axs[0].plot(t,ey,color=_C[1],lw=1.0,label=r"$e_y$")
    _vl(axs[0],tf); axs[0].set_ylabel("X/Y err (m)"); axs[0].legend(fontsize=7)
    axs[0].set_title("Load Tracking Error")
    axs[1].plot(t,np.sqrt(ex**2+ey**2),color=_C[2],lw=1.0); _vl(axs[1],tf); axs[1].set_ylabel("‖e_xy‖ (m)")
    axs[2].plot(t,np.abs(ez),color=_C[4],lw=1.0); _vl(axs[2],tf); axs[2].set_ylabel("|e_z| (m)")
    axs[3].plot(t,en,color=_C[5],lw=1.0); _vl(axs[3],tf); axs[3].set_ylabel("‖e‖ (m)")
    axs[3].set_xlabel("Time (s)")
    pre,post = t<=tf, t>tf
    if pre.any():
        rp = math.sqrt(np.mean(en[pre]**2))
        axs[3].axhline(rp,color=_C[0],ls=":",lw=0.8,label=f"Pre RMS={rp*100:.1f} cm")
    stable = post & (t>tf+3.)
    if stable.any():
        rs = math.sqrt(np.mean(en[stable]**2))
        axs[3].axhline(rs,color=_FAULT,ls=":",lw=0.8,label=f"Post RMS={rs*100:.1f} cm")
    axs[3].legend(fontsize=7); plt.tight_layout()
    _save(fig, os.path.join(out,"fig03_tracking_error.png"))

def fig04(df_tn, tf, fq, nq, out):
    if df_tn is None: return
    fig,ax = plt.subplots(figsize=(5.5,2.8)); t = df_tn["time"].values
    for q in range(nq):
        col = f"rope{q}_mag"
        if col not in df_tn.columns: continue
        v = df_tn[col].values.copy()
        if q==fq: v[t>tf]=0.
        c,ls = _ds(q,fq)
        ax.plot(t,v,color=c,ls=ls,lw=1.2,label=f"Cable {q}"+(" (severed)" if q==fq else ""))
    _vl(ax,tf,True); ax.set_xlabel("Time (s)"); ax.set_ylabel("Tension (N)")
    ax.set_title(f"Cable Tensions — N={nq} GPAC (fault t={tf:.0f} s)"); ax.legend(fontsize=7,ncol=2)
    plt.tight_layout(); _save(fig, os.path.join(out,"fig04_tensions.png"))

def fig05(df_t, ref, tf, out):
    t,z = df_t["time"].values, df_t["load_z"].values
    pre,post = t<=tf, t>tf
    fig,ax = plt.subplots(figsize=(5.5,2.6))
    ax.plot(t[pre],z[pre],color=_C[0],lw=1.2,label="Pre-fault")
    ax.plot(t[post],z[post],color=_FAULT,lw=1.2,label="Post-fault")
    if ref is not None: ax.plot(t,ref[:,2],color=_REF,lw=0.9,ls="--",label="Reference")
    if post.any():
        zm = np.mean(z[post][:min(200,post.sum())])
        ax.axhline(zm,color=_C[5],ls=":",lw=0.8,label=f"Post mean={zm:.2f} m")
    _vl(ax,tf,True); ax.set_xlabel("Time (s)"); ax.set_ylabel("Load Z (m)")
    ax.set_title("Load Altitude: Pre- vs Post-Fault"); ax.legend(fontsize=7)
    plt.tight_layout(); _save(fig, os.path.join(out,"fig05_altitude.png"))

def fig06(df_t, tf, fq, out):
    t = df_t["time"].values
    fig,axs = plt.subplots(3,1,figsize=(5.5,4.5),sharex=True)
    for ax,k,yl,c in zip(axs,["x","y","z"],["X (m)","Y (m)","Z (m)"],_C):
        col = f"drone{fq}_{k}"
        if col in df_t.columns: ax.plot(t,df_t[col].values,color=c,lw=1.2)
        _vl(ax,tf,k=="x"); ax.set_ylabel(yl)
    axs[0].set_title(f"Drone {fq} — Retreat Trajectory"); axs[0].legend(fontsize=7)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig06_fault_quad_retreat.png"))

def fig07(df_t, tf, fq, nq, out):
    t = df_t["time"].values
    fig,axs = plt.subplots(3,1,figsize=(5.5,5.5),sharex=True)
    for q in range(nq):
        c,ls = _ds(q,fq); lbl = f"Drone {q}"+(" (fault)" if q==fq else "")
        for ax,k in zip(axs,["x","y","z"]):
            col = f"drone{q}_{k}"
            if col in df_t.columns:
                ax.plot(t,df_t[col].values,color=c,ls=ls,lw=0.9,label=lbl if k=="x" else None)
    for ax,yl in zip(axs,["X (m)","Y (m)","Z (m)"]): _vl(ax,tf); ax.set_ylabel(yl)
    axs[0].set_title("All Drone Positions vs Time"); axs[0].legend(fontsize=7,ncol=2)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig07_all_drone_positions.png"))

def fig08(df_c, tf, fq, nq, out):
    if df_c is None: return
    fig,ax = plt.subplots(figsize=(5.5,2.8)); t = df_c["time"].values
    for q in range(nq):
        col = f"drone{q}_f_z"
        if col not in df_c.columns: continue
        c,ls = _ds(q,fq)
        ax.plot(t,df_c[col].values,color=c,ls=ls,lw=1.1,
                label=f"Drone {q}"+(" (fault)" if q==fq else ""))
    _vl(ax,tf,True); ax.set_xlabel("Time (s)"); ax.set_ylabel("Thrust Fz (N)")
    ax.set_title("Thrust Redistribution After Cable Fault"); ax.legend(fontsize=7,ncol=2)
    plt.tight_layout(); _save(fig, os.path.join(out,"fig08_thrust.png"))

def fig09(df_a, tf, fq, nq, out):
    if df_a is None: return
    t = df_a["time"].values
    fig,axs = plt.subplots(3,1,figsize=(5.5,5.5),sharex=True)
    for q in range(nq):
        c,ls = _ds(q,fq); lbl = f"Drone {q}"+(" (fault)" if q==fq else "")
        for ax,k in zip(axs,["roll","pitch","yaw"]):
            col = f"drone{q}_{k}"
            if col in df_a.columns:
                ax.plot(t,np.degrees(df_a[col].values),color=c,ls=ls,lw=0.9,
                        label=lbl if k=="roll" else None)
    for ax,yl in zip(axs,["Roll (°)","Pitch (°)","Yaw (°)"]): _vl(ax,tf); ax.set_ylabel(yl)
    axs[0].set_title("Drone Attitudes: Roll / Pitch / Yaw"); axs[0].legend(fontsize=7,ncol=2)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig09_attitudes.png"))

def fig10(df_a, tf, fq, nq, out):
    if df_a is None: return
    if not any(f"drone{q}_err_x" in df_a.columns for q in range(nq)):
        print("  ! No err_x in attitude_data — skipping fig10"); return
    t = df_a["time"].values
    fig,axs = plt.subplots(3,1,figsize=(5.5,5.0),sharex=True)
    for q in range(nq):
        c,ls = _ds(q,fq); lbl = f"Drone {q}"+(" (fault)" if q==fq else "")
        for ax,k in zip(axs,["err_x","err_y","err_z"]):
            col = f"drone{q}_{k}"
            if col in df_a.columns:
                ax.plot(t,df_a[col].values,color=c,ls=ls,lw=0.9,label=lbl if k=="err_x" else None)
    for ax,yl in zip(axs,["eR_x","eR_y","eR_z"]): _vl(ax,tf); ax.set_ylabel(yl)
    axs[0].set_title("Geometric Attitude Errors eR (GPAC Layer 2)"); axs[0].legend(fontsize=7,ncol=2)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig10_attitude_errors.png"))

def fig11(df_g, tf, fq, nq, pm, out):
    if df_g is None: return
    if not any(f"drone{q}_cl_mass" in df_g.columns for q in range(nq)):
        print("  ! No cl_mass in gpac_signals — skipping fig11"); return
    tt = pm/nq; t = df_g["time"].values
    fig,ax = plt.subplots(figsize=(5.5,2.8))
    for q in range(nq):
        col = f"drone{q}_cl_mass"
        if col not in df_g.columns: continue
        c,ls = _ds(q,fq)
        ax.plot(t,df_g[col].values,color=c,ls=ls,lw=1.2,
                label=rf"Drone {q} $\hat\theta$"+(" (fault)" if q==fq else ""))
    ax.axhline(tt,color="black",ls=":",lw=1.0,label=f"True θ={tt:.3f} kg")
    _vl(ax,tf,True); ax.set_xlabel("Time (s)"); ax.set_ylabel("θ̂ (kg)")
    ax.set_title(r"Concurrent-Learning Mass Estimates $\hat{\theta}_i = m_L / N$")
    ax.legend(fontsize=7,ncol=2); plt.tight_layout()
    _save(fig, os.path.join(out,"fig11_mass_estimates.png"))

def fig12(df_g, tf, fq, nq, out):
    if df_g is None: return
    if not any(f"drone{q}_eso_dx" in df_g.columns for q in range(nq)):
        print("  ! No eso_dx — skipping fig12"); return
    t = df_g["time"].values
    fig,axs = plt.subplots(3,1,figsize=(5.5,5.0),sharex=True)
    for q in range(nq):
        c,ls = _ds(q,fq); lbl = f"Drone {q}"+(" (fault)" if q==fq else "")
        for ax,k in zip(axs,["eso_dx","eso_dy","eso_dz"]):
            col = f"drone{q}_{k}"
            if col in df_g.columns:
                ax.plot(t,df_g[col].values,color=c,ls=ls,lw=0.9,label=lbl if k=="eso_dx" else None)
    for ax,yl in zip(axs,["d̂_x (N)","d̂_y (N)","d̂_z (N)"]): _vl(ax,tf); ax.set_ylabel(yl)
    axs[0].set_title("ESO Disturbance Estimates (GPAC Layer 4)"); axs[0].legend(fontsize=7,ncol=2)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig12_eso_disturbance.png"))

def fig13(df_g, tf, fq, nq, out):
    if df_g is None: return
    if not any(f"drone{q}_antiswing_fx" in df_g.columns for q in range(nq)):
        print("  ! No antiswing_fx — skipping fig13"); return
    t = df_g["time"].values
    fig,axs = plt.subplots(3,1,figsize=(5.5,5.0),sharex=True)
    for q in range(nq):
        c,ls = _ds(q,fq); lbl = f"Drone {q}"+(" (fault)" if q==fq else "")
        for ax,k in zip(axs,["antiswing_fx","antiswing_fy","antiswing_fz"]):
            col = f"drone{q}_{k}"
            if col in df_g.columns:
                ax.plot(t,df_g[col].values,color=c,ls=ls,lw=0.9,label=lbl if k=="antiswing_fx" else None)
    for ax,yl in zip(axs,["f_asw_x (N)","f_asw_y (N)","f_asw_z (N)"]): _vl(ax,tf); ax.set_ylabel(yl)
    axs[0].set_title("Anti-Swing Cable Forces on S² (GPAC Layer 1)"); axs[0].legend(fontsize=7,ncol=2)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig13_antiswing_forces.png"))

def fig14(df_g, tf, fq, nq, out):
    if df_g is None: return
    keys = [k for k in ["cbf_taut_lo","cbf_taut_hi","cbf_angle","cbf_swing","cbf_tilt","cbf_collision"]
            if f"drone0_{k}" in df_g.columns]
    if not keys: print("  ! No CBF columns — skipping fig14"); return
    t = df_g["time"].values
    fig,axs = plt.subplots(len(keys),1,figsize=(5.5,1.5*len(keys)),sharex=True)
    if len(keys)==1: axs=[axs]
    for ax,k in zip(axs,keys):
        for q in range(nq):
            col = f"drone{q}_{k}"
            if col in df_g.columns:
                c,ls = _ds(q,fq)
                ax.plot(t,df_g[col].values,color=c,ls=ls,lw=0.9,
                        label=f"Drone {q}" if k==keys[0] else None)
        ax.axhline(0.,color="black",ls=":",lw=0.7); _vl(ax,tf)
        ax.set_ylabel(k.replace("cbf_","h_"),fontsize=7)
    axs[0].set_title("CBF Safety Barrier Values (h > 0 = safe)"); axs[0].legend(fontsize=6,ncol=2)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig14_cbf_barriers.png"))

def fig15(df_g, tf, fq, nq, out):
    if df_g is None: return
    if not any(f"drone{q}_cable_qx" in df_g.columns for q in range(nq)):
        print("  ! No cable_qx — skipping fig15"); return
    t = df_g["time"].values
    fig,axs = plt.subplots(3,1,figsize=(5.5,5.0),sharex=True)
    for q in range(nq):
        c,ls = _ds(q,fq); lbl = f"Drone {q}"+(" (fault)" if q==fq else "")
        for ax,k in zip(axs,["cable_qx","cable_qy","cable_qz"]):
            col = f"drone{q}_{k}"
            if col in df_g.columns:
                ax.plot(t,df_g[col].values,color=c,ls=ls,lw=0.9,label=lbl if k=="cable_qx" else None)
    for ax,yl in zip(axs,["n_x","n_y","n_z"]): _vl(ax,tf); ax.set_ylabel(yl)
    axs[0].set_title("Cable Unit Vectors on S² (load→drone)"); axs[0].legend(fontsize=7,ncol=2)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig15_cable_direction.png"))

def fig16(df_e, df_t, tf, fq, nq, out):
    if df_e is None: return
    te,tt = df_e["time"].values, df_t["time"].values
    fig,axs = plt.subplots(3,1,figsize=(5.5,5.0),sharex=True)
    for q in range(nq):
        c,ls = _ds(q,fq); lbl = f"Drone {q}"+(" (fault)" if q==fq else "")
        for ax,k in zip(axs,["x","y","z"]):
            ce,ct = f"drone{q}_est_{k}", f"drone{q}_{k}"
            if ce not in df_e.columns or ct not in df_t.columns: continue
            ax.plot(te, df_e[ce].values, color=c, ls=ls, lw=0.9, label=f"Est {lbl}" if k=="x" else None)
            ax.plot(te, np.interp(te,tt,df_t[ct].values), color=c, ls=":", lw=0.6, alpha=0.6,
                    label=f"True {lbl}" if k=="x" else None)
    for ax,yl in zip(axs,["X (m)","Y (m)","Z (m)"]): _vl(ax,tf); ax.set_ylabel(yl)
    axs[0].set_title("ESKF Estimated vs True Drone Positions"); axs[0].legend(fontsize=5,ncol=4)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig16_eskf_accuracy.png"))

def fig17(df_t, tf, out):
    if "load_qx" not in df_t.columns: print("  ! No load quaternion — skipping fig17"); return
    t  = df_t["time"].values
    qx,qy = df_t["load_qx"].values, df_t["load_qy"].values
    tilt = np.degrees(np.arccos(np.clip(1.-2.*(qx**2+qy**2),-1.,1.)))
    pre,post = t<=tf, t>tf
    fig,ax = plt.subplots(figsize=(5.5,2.4))
    ax.plot(t[pre],tilt[pre],color=_C[0],lw=1.2,label="Pre-fault")
    ax.plot(t[post],tilt[post],color=_FAULT,lw=1.2,label="Post-fault")
    _vl(ax,tf,True); ax.set_xlabel("Time (s)"); ax.set_ylabel("Load Tilt (°)")
    ax.set_title("Load Swing Tilt from Vertical"); ax.legend(fontsize=7)
    plt.tight_layout(); _save(fig, os.path.join(out,"fig17_load_swing.png"))

def fig18(df_tn, tf, fq, nq, out):
    if df_tn is None or "rope0_fx" not in df_tn.columns:
        print("  ! No rope force vectors — skipping fig18"); return
    t = df_tn["time"].values
    fig,ax = plt.subplots(figsize=(5.5,2.6))
    for q in range(nq):
        if f"rope{q}_fz" not in df_tn.columns: continue
        fx,fy,fz = (df_tn[f"rope{q}_f{k}"].values for k in "xyz")
        el = np.degrees(np.arctan2(np.abs(fz),np.sqrt(fx**2+fy**2))).astype(float)
        if q==fq: el[t>tf]=np.nan
        c,ls = _ds(q,fq)
        ax.plot(t,el,color=c,ls=ls,lw=1.1,label=f"Cable {q}"+(" (cut)" if q==fq else ""))
    _vl(ax,tf,True); ax.set_xlabel("Time (s)"); ax.set_ylabel("Elevation (°)")
    ax.set_title("Cable Force Elevation Angles"); ax.legend(fontsize=6,ncol=2)
    plt.tight_layout(); _save(fig, os.path.join(out,"fig18_cable_elevation.png"))

def fig19(df_t, df_tn, ref, tf, fq, nq, out):
    t=df_t["time"].values; lo,hi=max(t[0],tf-6.),min(t[-1],tf+14.)
    mk=((t>=lo)&(t<=hi)); tm=t[mk]
    fig,axs=plt.subplots(3,1,figsize=(5.5,5.5),sharex=True)
    axs[0].plot(tm,df_t["load_x"].values[mk],color=_C[0],lw=1.2,label="Load X")
    axs[0].plot(tm,df_t["load_y"].values[mk],color=_C[1],lw=1.0,label="Load Y")
    if ref is not None: axs[0].plot(tm,ref[mk,0],color=_REF,lw=0.8,ls="--",label="Ref X")
    _vl(axs[0],tf); axs[0].set_ylabel("Position (m)")
    axs[0].set_title(f"Fault Transient Detail (t_fault={tf:.0f} s)"); axs[0].legend(fontsize=7)
    if ref is not None:
        ex=df_t["load_x"].values[mk]-ref[mk,0]; ey=df_t["load_y"].values[mk]-ref[mk,1]
        en=np.sqrt(ex**2+ey**2)
        axs[1].plot(tm,en,color=_C[2],lw=1.2); axs[1].fill_between(tm,0,en,color=_C[2],alpha=0.2)
    _vl(axs[1],tf); axs[1].set_ylabel("‖e_xy‖ (m)")
    if df_tn is not None:
        tt=df_tn["time"].values; mk2=((tt>=lo)&(tt<=hi)); t2=tt[mk2]
        for q in range(nq):
            col=f"rope{q}_mag"
            if col not in df_tn.columns: continue
            v=df_tn[col].values[mk2].copy()
            if q==fq: v[t2>tf]=0.
            c,ls=_ds(q,fq)
            axs[2].plot(t2,v,color=c,ls=ls,lw=1.1,label=f"Cable {q}"+(" (cut)" if q==fq else ""))
    _vl(axs[2],tf); axs[2].set_ylabel("Tension (N)"); axs[2].legend(fontsize=6,ncol=2)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig19_transient_zoom.png"))

def fig20(df_c, tf, fq, nq, out):
    if df_c is None: return
    fig,axs=plt.subplots(3,1,figsize=(5.5,5.0),sharex=True); t=df_c["time"].values
    for q in range(nq):
        c,ls=_ds(q,fq); lbl=f"Drone {q}"+(" (fault)" if q==fq else "")
        for ax,k in zip(axs,["tau_x","tau_y","tau_z"]):
            col=f"drone{q}_{k}"
            if col in df_c.columns:
                ax.plot(t,df_c[col].values,color=c,ls=ls,lw=0.9,label=lbl if k=="tau_x" else None)
    for ax,yl in zip(axs,["τ_x (N·m)","τ_y (N·m)","τ_z (N·m)"]): _vl(ax,tf); ax.set_ylabel(yl)
    axs[0].set_title("Control Torques per Drone"); axs[0].legend(fontsize=7,ncol=2)
    axs[-1].set_xlabel("Time (s)"); plt.tight_layout()
    _save(fig, os.path.join(out,"fig20_torques.png"))

def fig21(df_t, ref, tf, out):
    t=df_t["time"].values; x,y=df_t["load_x"].values,df_t["load_y"].values
    pre,post=t<=tf,t>tf
    fig,ax=plt.subplots(figsize=(3.5,3.5))
    if ref is not None: ax.plot(ref[:,0],ref[:,1],color=_REF,lw=0.9,ls="--",label="Reference")
    ax.plot(x[pre],y[pre],color=_C[0],lw=1.2,label="Pre-fault")
    ax.plot(x[post],y[post],color=_FAULT,lw=1.2,label="Post-fault")
    if pre.any(): ax.scatter(x[pre][-1],y[pre][-1],marker="x",s=60,color=_FAULT,zorder=5)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_title("Load XY Overhead Path")
    ax.set_aspect("equal","box"); ax.legend(fontsize=7)
    plt.tight_layout(); _save(fig, os.path.join(out,"fig21_xy_path.png"))

def fig22(df_t, df_tn, df_g, ref, tf, fq, nq, pm, out):
    fig=plt.figure(figsize=(7.0,8.0)); gs=gridspec.GridSpec(4,2,figure=fig,hspace=0.50,wspace=0.36)
    t=df_t["time"].values; z=df_t["load_z"].values

    ax_a=fig.add_subplot(gs[0,0])
    ax_a.plot(t,z,color=_C[0],lw=1.1,label="Actual")
    if ref is not None: ax_a.plot(t,ref[:,2],color=_REF,lw=0.9,ls="--",label="Ref")
    _vl(ax_a,tf); ax_a.set_ylabel("Load Z (m)"); ax_a.set_title("(a) Altitude"); ax_a.legend(fontsize=6)

    ax_b=fig.add_subplot(gs[0,1])
    if ref is not None:
        en=np.sqrt((df_t["load_x"].values-ref[:,0])**2+(df_t["load_y"].values-ref[:,1])**2+(z-ref[:,2])**2)
        ax_b.plot(t,en,color=_C[5],lw=1.1)
    _vl(ax_b,tf); ax_b.set_ylabel("‖e‖ (m)"); ax_b.set_title("(b) Tracking Error")

    ax_c=fig.add_subplot(gs[1,:])
    if df_tn is not None:
        tt=df_tn["time"].values
        for q in range(nq):
            col=f"rope{q}_mag"
            if col not in df_tn.columns: continue
            v=df_tn[col].values.copy()
            if q==fq: v[tt>tf]=0.
            c,ls=_ds(q,fq)
            ax_c.plot(tt,v,color=c,ls=ls,lw=1.0,label=f"Cable {q}"+(" (cut)" if q==fq else ""))
    _vl(ax_c,tf); ax_c.set_ylabel("Tension (N)"); ax_c.set_title("(c) Cable Tensions"); ax_c.legend(fontsize=6,ncol=2)

    ax_d=fig.add_subplot(gs[2,0]); true_t=pm/nq
    if df_g is not None:
        tg=df_g["time"].values
        for q in range(nq):
            col=f"drone{q}_cl_mass"
            if col in df_g.columns:
                c,ls=_ds(q,fq); ax_d.plot(tg,df_g[col].values,color=c,ls=ls,lw=1.0,label=rf"$\hat\theta_{q}$")
    ax_d.axhline(true_t,color="black",ls=":",lw=0.9,label=f"True={true_t:.3f} kg")
    _vl(ax_d,tf); ax_d.set_ylabel("θ̂ (kg)"); ax_d.set_title("(d) Mass Estimates"); ax_d.legend(fontsize=6,ncol=2)

    ax_e=fig.add_subplot(gs[2,1]); pre,post=t<=tf,t>tf
    if ref is not None: ax_e.plot(ref[:,0],ref[:,1],color=_REF,lw=0.8,ls="--",label="Ref")
    ax_e.plot(df_t["load_x"].values[pre],df_t["load_y"].values[pre],color=_C[0],lw=1.0,label="Pre-fault")
    ax_e.plot(df_t["load_x"].values[post],df_t["load_y"].values[post],color=_FAULT,lw=1.0,label="Post-fault")
    ax_e.set_xlabel("X (m)"); ax_e.set_ylabel("Y (m)"); ax_e.set_title("(e) XY Path")
    ax_e.set_aspect("equal","box"); ax_e.legend(fontsize=6)

    ax_f=fig.add_subplot(gs[3,:])
    for q in range(nq):
        col=f"drone{q}_z"
        if col in df_t.columns:
            c,ls=_ds(q,fq)
            ax_f.plot(t,df_t[col].values,color=c,ls=ls,lw=0.9,
                      label=f"Drone {q}"+(" (fault)" if q==fq else ""))
    _vl(ax_f,tf); ax_f.set_ylabel("Drone Z (m)"); ax_f.set_title("(f) Drone Altitudes")
    ax_f.set_xlabel("Time (s)"); ax_f.legend(fontsize=6,ncol=2)

    fig.suptitle(f"GPAC N={nq} Cable-Snap Fault — Summary Dashboard",fontsize=10,y=1.005)
    _save(fig, os.path.join(out,"fig22_summary_dashboard.png"))

# ── Numerical summary ────────────────────────────────────────────────────────
def _summary(df_t, df_tn, ref, tf, fq, nq, pm, out):
    t,z=df_t["time"].values,df_t["load_z"].values
    pre,post=t<=tf,t>tf; L=[]
    def P(s=""):  print(s); L.append(s)
    P("="*58); P(f"GPAC N={nq} Cable-Fault Simulation — Numerical Summary"); P("="*58)
    P(f"  Fault cable  : {fq}  (cut at t={tf:.1f} s)")
    P(f"  Payload mass : {pm:.2f} kg  (θ_true/drone = {pm/nq:.4f} kg)")
    P(f"  Sim end      : {t[-1]:.1f} s")
    if pre.any(): P(f"\n  Load Z pre-fault  : mean={np.mean(z[pre]):.3f} m, std={np.std(z[pre])*100:.2f} cm")
    if post.any():
        P(f"  Load Z post-fault : mean={np.mean(z[post]):.3f} m, std={np.std(z[post])*100:.2f} cm")
        P(f"  Load Z drop       : {z[post][0]-z[pre][-1]:+.3f} m → final {z[-1]:.3f} m")
    if ref is not None:
        en=np.sqrt((df_t["load_x"].values-ref[:,0])**2+(df_t["load_y"].values-ref[:,1])**2+(z-ref[:,2])**2)
        if pre.any(): P(f"\n  Tracking error pre-fault  RMS : {np.sqrt(np.mean(en[pre]**2))*100:.1f} cm")
        st=post&(t>tf+3.)
        if st.any(): P(f"  Tracking error post-fault RMS : {np.sqrt(np.mean(en[st]**2))*100:.1f} cm (after +3 s)")
        if post.any(): P(f"  Peak error at transit        : {np.max(en[post][:100])*100:.1f} cm")
    if df_tn is not None:
        P("\n  Cable tension stats:")
        tt=df_tn["time"].values
        for q in range(nq):
            col=f"rope{q}_mag"
            if col not in df_tn.columns: continue
            vp=df_tn[col].values[tt<=tf]; va=df_tn[col].values[tt>tf]
            if q==fq: P(f"    Cable {q} (SEVERED): pre mean={np.mean(vp):.2f} N → 0 N")
            elif va.size: P(f"    Cable {q}          : pre {np.mean(vp):.2f} N → post {np.mean(va):.2f} N (Δ={np.mean(va)-np.mean(vp):+.2f} N)")
    P("="*58)
    with open(os.path.join(out,"numerical_summary.txt"),"w") as f: f.write("\n".join(L)+"\n")
    print(f"  + numerical_summary.txt")

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--log-dir",       required=True)
    ap.add_argument("--fault-time",    type=float, default=20.0)
    ap.add_argument("--fault-quad",    type=int,   default=0)
    ap.add_argument("--num-quads",     type=int,   default=4)
    ap.add_argument("--payload-mass",  type=float, default=3.0)
    ap.add_argument("--output-dir",    default=None)
    args=ap.parse_args()

    if not os.path.isdir(args.log_dir):
        sys.exit(f"ERROR: --log-dir not found: {args.log_dir}")

    out=args.output_dir or str(Path(args.log_dir).parent.parent/"analysis_gpac")
    os.makedirs(out,exist_ok=True)
    print(f"\nLog     : {args.log_dir}\nFigures : {out}\n")
    _ieee()

    df_t  = _csv(args.log_dir,"trajectories.csv",        req=True)
    df_tn = _csv(args.log_dir,"tensions.csv")
    df_c  = _csv(args.log_dir,"control_efforts.csv")
    df_e  = _csv(args.log_dir,"estimator_outputs.csv")
    df_a  = _csv(args.log_dir,"attitude_data.csv")
    df_g  = _csv(args.log_dir,"gpac_signals.csv")

    cfg  = _cfg(args.log_dir)
    wps  = _wps(cfg)
    ref  = _build_ref(df_t["time"].values, wps) if wps else None
    pm   = float(cfg.get("payload_mass",  args.payload_mass))
    tf   = float(cfg.get("fault_time",    args.fault_time))
    fq   = int(cfg.get("fault_quad_index",args.fault_quad))
    nq   = int(cfg.get("num_quadcopters", args.num_quads))

    print(f"\n  Config: {len(wps)} waypoints, fault cable {fq} at t={tf:.1f} s, N={nq}, m_L={pm} kg\n")
    _summary(df_t,df_tn,ref,tf,fq,nq,pm,out)
    print("\nGenerating figures...")

    fig01(df_t,tf,out)
    fig02(df_t,ref,tf,out)
    fig03(df_t,ref,tf,out)
    fig04(df_tn,tf,fq,nq,out)
    fig05(df_t,ref,tf,out)
    fig06(df_t,tf,fq,out)
    fig07(df_t,tf,fq,nq,out)
    fig08(df_c,tf,fq,nq,out)
    fig09(df_a,tf,fq,nq,out)
    fig10(df_a,tf,fq,nq,out)
    fig11(df_g,tf,fq,nq,pm,out)
    fig12(df_g,tf,fq,nq,out)
    fig13(df_g,tf,fq,nq,out)
    fig14(df_g,tf,fq,nq,out)
    fig15(df_g,tf,fq,nq,out)
    fig16(df_e,df_t,tf,fq,nq,out)
    fig17(df_t,tf,out)
    fig18(df_tn,tf,fq,nq,out)
    fig19(df_t,df_tn,ref,tf,fq,nq,out)
    fig20(df_c,tf,fq,nq,out)
    fig21(df_t,ref,tf,out)
    fig22(df_t,df_tn,df_g,ref,tf,fq,nq,pm,out)

    total=len(os.listdir(out))
    print(f"\nDone — {total} files in {out}\n")

if __name__=="__main__":
    main()
