#!/usr/bin/env python3
"""Graphical-abstract redesign — data-driven, spacious layout.

Uses raw Drake screenshots and generates plots from NPZ data.
"""

from PIL import Image, ImageDraw, ImageFont, ImageFilter
import os
import numpy as np

# ── Paths ─────────────────────────────────────────────────────────────

ROOT = '/workspaces/Tether_Grace'
FIG_DIR = os.path.join(ROOT, 'IEEE_T-CST', 'Figures')
DRAKE_DIR = os.path.join(FIG_DIR, 'full_drake', 'seven_drones')
OUT_PNG = os.path.join(FIG_DIR, 'graphical_abstract.png')
OUT_PDF = os.path.join(FIG_DIR, 'graphical_abstract.pdf')

NPZ_PATHS = {
    3: os.path.join(ROOT, 'outputs/full_drake_fault_batch/three_drones/full_drake_recording.npz'),
    5: os.path.join(ROOT, 'outputs/full_drake_fault_batch/five_drones/full_drake_recording.npz'),
    7: os.path.join(ROOT, 'outputs/full_drake_fault_batch/seven_drones/full_drake_recording.npz'),
}

DRAKE_IMAGES = {
    'nominal':  os.path.join(DRAKE_DIR, '7-drones_1.png'),
    'snap':     os.path.join(DRAKE_DIR, '7-drones_2.png'),
    'recovery': os.path.join(DRAKE_DIR, '7-drones_7.png'),
}

SCENARIO_CONFIGS = {
    3: {'fault_cables': [0],       'fault_times': [7.0],              'duration': 30.0},
    5: {'fault_cables': [1, 3],    'fault_times': [7.0, 12.0],       'duration': 30.0},
    7: {'fault_cables': [1, 3, 5], 'fault_times': [7.0, 12.0, 14.0], 'duration': 30.0},
}
FAULT_CABLES = [1, 3, 5]
FAULT_TIMES = [7.0, 12.0, 14.0]
RMSE_VALUES = {3: 48.84, 5: 46.37, 7: 43.60}
COLORS_MPL = {3: '#1f77b4', 5: '#ff7f0e', 7: '#2ca02c'}

# ── Canvas ────────────────────────────────────────────────────────────

W, H = 3200, 5200
MARGIN = 120
BG = '#f6f2eb'
NAVY = '#10263d'
GREEN = '#2e8b57'
RED = '#d9534f'
GOLD = '#d9a44b'
MUTED = '#5f6771'
OFFWHITE = '#ffffff'
LIGHT_BLUE = '#eef3f8'
LIGHT_GOLD = '#fbf7ef'

FONT_DIR = '/usr/share/fonts/truetype/dejavu'


def fnt(name, size):
    return ImageFont.truetype(os.path.join(FONT_DIR, name), size)


F = {
    'title':      fnt('DejaVuSans-Bold.ttf', 88),
    'subtitle':   fnt('DejaVuSans.ttf', 38),
    'section':    fnt('DejaVuSans-Bold.ttf', 52),
    'label':      fnt('DejaVuSans-Bold.ttf', 36),
    'body':       fnt('DejaVuSans.ttf', 32),
    'small':      fnt('DejaVuSans.ttf', 30),
    'small_bold': fnt('DejaVuSans-Bold.ttf', 30),
    'equation':   fnt('DejaVuSans-Bold.ttf', 56),
    'arrow':      fnt('DejaVuSans-Bold.ttf', 88),
    'pill':       fnt('DejaVuSans-Bold.ttf', 28),
}


# ── Helpers ───────────────────────────────────────────────────────────

def card(base, box, radius=30, fill=OFFWHITE, outline=None, width=1, shadow=True):
    if shadow:
        sl = Image.new('RGBA', base.size, (0,0,0,0))
        sd = ImageDraw.Draw(sl)
        x1,y1,x2,y2 = box
        sd.rounded_rectangle((x1+12,y1+14,x2+12,y2+14), radius, fill=(20,30,40,40))
        sl = sl.filter(ImageFilter.GaussianBlur(14))
        base.alpha_composite(sl)
    ImageDraw.Draw(base).rounded_rectangle(box, radius, fill=fill, outline=outline, width=width)


def wrap(text, draw, font_obj, max_w):
    words = text.split(); lines = []; cur = ''
    for w in words:
        t = (cur+' '+w).strip()
        if draw.textbbox((0,0), t, font=font_obj)[2] <= max_w: cur = t
        else:
            if cur: lines.append(cur)
            cur = w
    if cur: lines.append(cur)
    return '\n'.join(lines)


def dwrap(draw, xy, text, font_obj, fill, max_w, spacing=8, align='left'):
    wrapped = wrap(text, draw, font_obj, max_w)
    x, y = int(xy[0]), int(xy[1])
    if align == 'center':
        bb = draw.multiline_textbbox((0,0), wrapped, font=font_obj, spacing=spacing, align=align)
        x = x - (bb[2]-bb[0])//2
    draw.multiline_text((x,y), wrapped, font=font_obj, fill=fill, spacing=spacing, align=align)
    bb = draw.multiline_textbbox((x,y), wrapped, font=font_obj, spacing=spacing, align=align)
    return bb[3]  # return bottom y


def pill(base, x, y, text, color, height=66):
    d = ImageDraw.Draw(base)
    tw = d.textbbox((0,0), text, font=F['pill'])[2]
    w = tw + 44
    d.rounded_rectangle((x, y, x+w, y+height), height//2, fill='white', outline=color, width=3)
    d.text((x+22, y + (height-28)//2), text, font=F['pill'], fill=color)
    return x + w


def paste_img(base, path, box, border_color=None, bw=5):
    img = Image.open(path).convert('RGBA')
    bx,by,bx2,by2 = box
    img.thumbnail((bx2-bx-2*bw, by2-by-2*bw), Image.LANCZOS)
    px = bx + (bx2-bx-img.width)//2
    py = by + (by2-by-img.height)//2
    if border_color:
        ImageDraw.Draw(base).rounded_rectangle(
            (px-bw, py-bw, px+img.width+bw, py+img.height+bw),
            18, outline=border_color, width=bw)
    base.alpha_composite(img, (px, py))


def mpl_panel(render_fn, w_px, h_px):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    dpi = 150
    fig, ax = plt.subplots(figsize=(w_px/dpi, h_px/dpi), dpi=dpi)
    render_fn(ax)
    fig.tight_layout(pad=0.8)
    fig.canvas.draw()
    img = Image.frombuffer('RGBA', fig.canvas.get_width_height(), fig.canvas.buffer_rgba())
    plt.close(fig)
    return img


# ── Reference trajectory ─────────────────────────────────────────────

def build_ref(times, dur):
    ts = [f*dur for f in [0.08,0.18,0.28,0.38,0.48,0.58,0.68,0.78,0.85,0.90]]
    pts = [[0,0,1.2],[0,0,2.8],[1.6,0.5,3.0],[2.4,1.4,3.2],[3.0,0,3.1],
           [2.1,-1.4,2.9],[0,0,3.0],[-1.6,0.5,3.1],[-2.6,1.4,3.3],
           [-3.0,0,3.1],[-2.0,-1.2,2.9],[0,0,2.6]]
    holds = [1.0,0.4,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.0]
    arrivals = [0.0]+ts+[dur]
    wps = [(np.array(p),a,h) for p,a,h in zip(pts,arrivals,holds)]
    ref = np.zeros((len(times),3))
    for i,t in enumerate(times):
        t = float(t); seg_s = 0.0
        for j,(pos,arr,hold) in enumerate(wps):
            if t <= arr:
                if j==0: ref[i]=pos; break
                prev=wps[j-1][0]; d=arr-seg_s
                ref[i] = (1-(t-seg_s)/d)*prev+(t-seg_s)/d*pos if d>1e-6 else pos; break
            if t <= arr+hold: ref[i]=pos; break
            seg_s = arr+hold
        else: ref[i]=wps[-1][0]
    return ref

def get_error(path, ft0, dur):
    npz=np.load(path,allow_pickle=True); h=list(npz['trajectory_headers']); tr=npz['trajectories']
    t=tr[:,h.index('time')]; xyz=tr[:,[h.index('load_x'),h.index('load_y'),h.index('load_z')]]
    ref=build_ref(t,dur)
    ws=min(2.0,max(float(t[0]),ft0*0.25)); we=max(ws+0.5,ft0-0.5)
    m=(t>=ws)&(t<=we); zo=float(np.median(ref[m,2]-xyz[m,2])) if np.any(m) else 0
    ref[:,2]-=zo; return t, np.linalg.norm(xyz-ref,axis=1)*100, xyz, ref

def get_tensions(path, n):
    npz=np.load(path,allow_pickle=True); h=list(npz['tension_headers']); te=npz['tensions']
    t=te[:,h.index('time')]; return t,[te[:,h.index(f'rope{i}_mag')] for i in range(n)]


# ── Plot renderers (no titles — titles are drawn on the canvas) ──────

def plot_error(ax):
    for n in [3,5,7]:
        cfg=SCENARIO_CONFIGS[n]; t,err,_,_=get_error(NPZ_PATHS[n],cfg['fault_times'][0],cfg['duration'])
        nf=len(cfg['fault_cables'])
        ax.plot(t,err,color=COLORS_MPL[n],lw=2.5,label=f'N={n}, {nf} fault{"s" if nf>1 else ""}')
        for ft in cfg['fault_times']: ax.axvline(ft,color=COLORS_MPL[n],ls='--',lw=1,alpha=0.4)
    ax.axvspan(7,30,alpha=0.04,color='red')
    ax.set_xlim(3,30); ax.set_ylim(0,140)
    ax.set_xlabel('Time [s]',fontsize=14); ax.set_ylabel('Error [cm]',fontsize=14)
    ax.legend(fontsize=13,loc='upper right'); ax.grid(True,alpha=0.2,ls='--'); ax.tick_params(labelsize=12)

def plot_tensions(ax):
    t,tens=get_tensions(NPZ_PATHS[7],7)
    cc=['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd','#8c564b','#e377c2']
    for i in range(7):
        f=i in FAULT_CABLES
        ax.plot(t,tens[i],color=cc[i],lw=1.5 if f else 1.1,alpha=0.85,
                label=f'Cable {i}'+(' — faulted' if f else ''))
    for ft in FAULT_TIMES: ax.axvline(ft,color='#d62728',ls='--',lw=1,alpha=0.5)
    ax.axvspan(7,30,alpha=0.04,color='red')
    ax.set_xlim(3,30); ax.set_ylim(-0.5,22)
    ax.set_xlabel('Time [s]',fontsize=14); ax.set_ylabel('Tension [N]',fontsize=14)
    ax.legend(fontsize=9,ncol=2,loc='upper left'); ax.grid(True,alpha=0.2,ls='--'); ax.tick_params(labelsize=12)

def plot_xy(ax):
    t,_,xyz,ref=get_error(NPZ_PATHS[7],7.0,30.0); m=t>=2.0
    ax.plot(ref[m,0],ref[m,1],'--',color='#888',lw=2.5,label='Reference')
    ax.plot(xyz[m,0],xyz[m,1],'-',color='#2ca02c',lw=2,alpha=0.9,label='Actual')
    for i,ft in enumerate(FAULT_TIMES):
        idx=np.argmin(np.abs(t-ft))
        ax.plot(xyz[idx,0],xyz[idx,1],'x',color='#d62728',ms=14,mew=3,
                label='Cable snap' if i==0 else None)
    ax.set_aspect('equal',adjustable='datalim')
    ax.set_xlabel('x [m]',fontsize=14); ax.set_ylabel('y [m]',fontsize=14)
    ax.legend(fontsize=12,loc='upper left'); ax.grid(True,alpha=0.2,ls='--'); ax.tick_params(labelsize=12)

def plot_rmse(ax):
    teams=[3,5,7]; vals=[RMSE_VALUES[n] for n in teams]
    bars=ax.bar(range(3),vals,color=[COLORS_MPL[n] for n in teams],width=0.6,edgecolor='white')
    for b,v in zip(bars,vals):
        ax.text(b.get_x()+b.get_width()/2,b.get_height()+0.8,f'{v:.1f}',
                ha='center',fontsize=14,fontweight='bold',color='#333')
    ax.set_xticks(range(3)); ax.set_xticklabels(['N=3\n1 fault','N=5\n2 faults','N=7\n3 faults'],fontsize=12)
    ax.set_ylabel('RMSE [cm]',fontsize=14); ax.set_ylim(0,65)
    ax.grid(True,axis='y',alpha=0.2,ls='--'); ax.tick_params(labelsize=12)
    ax.annotate('11% lower\ndespite 3× more faults',xy=(2,44),xytext=(0.6,16),
                fontsize=13,ha='center',color='#2ca02c',fontweight='bold',
                arrowprops=dict(arrowstyle='->',color='#2ca02c',lw=2))


# ══════════════════════════════════════════════════════════════════════
# BUILD
# ══════════════════════════════════════════════════════════════════════

def build():
    canvas = Image.new('RGBA', (W, H), BG)
    d = ImageDraw.Draw(canvas)
    M = MARGIN
    CW = W - 2*M  # content width

    y = 0  # running y cursor

    # ══════════════════════════════════════════════════════════════════
    # HEADER
    # ══════════════════════════════════════════════════════════════════
    header_h = 400
    card(canvas, (M-20, M-20, W-M+20, M+header_h), radius=36, fill=NAVY)
    d = ImageDraw.Draw(canvas)
    d.text((M+40, M+30), 'Inherent Cable-Fault Tolerance', font=F['title'], fill='white')
    d.text((M+40, M+130), 'in Decentralized Cooperative Aerial Transport',
           font=F['title'], fill='white')
    dwrap(d, (M+40, M+260),
          'When cables snap mid-flight, the remaining drones absorb the fault '
          'and keep tracking the payload \u2014 with no fault detection, '
          'no communication, and no reconfiguration.',
          F['subtitle'], '#d0dce8', CW-80, spacing=10)
    y = M + header_h + 30

    # Pills
    x = M
    x = pill(canvas, x, y, 'No fault detection', RED)
    x = pill(canvas, x+24, y, 'No communication', NAVY)
    x = pill(canvas, x+24, y, 'No reconfiguration', GOLD)
    pill(canvas, x+24, y, '43.6 cm RMSE', GREEN)
    y += 100

    # ══════════════════════════════════════════════════════════════════
    # SECTION 1: Three-stage recovery
    # ══════════════════════════════════════════════════════════════════
    d = ImageDraw.Draw(canvas)
    d.text((M, y), '1   Three-stage recovery sequence', font=F['section'], fill=NAVY)
    y += 80

    story = [
        ('01', GREEN, 'Nominal operation',
         'Seven quadrotors carry a cable-suspended\n'
         'payload in heptagonal formation. No drone\n'
         'knows the team size or other agents\' states.',
         'nominal'),
        ('02', RED, 'Cable snap',
         'At t = 7 s, cable 1 snaps. The freed drone\n'
         'accelerates away. Remaining drones get\n'
         'a sudden 17% load-share increase.',
         'snap'),
        ('03', GREEN, 'Stable recovery',
         'After 3 cascading snaps (t = 7, 12, 14 s),\n'
         '4 surviving drones keep tracking with\n'
         'bounded error (43.6 cm RMSE).',
         'recovery'),
    ]

    cw = (CW - 80) // 3  # card width
    ch = 660             # card height
    img_h = 340          # image area height inside card

    for i, (step, color, title, body, key) in enumerate(story):
        cx = M + i*(cw+40)
        card(canvas, (cx, y, cx+cw, y+ch), fill=OFFWHITE)
        d = ImageDraw.Draw(canvas)

        # Step badge
        d.rounded_rectangle((cx+24, y+20, cx+150, y+72), 26, fill=color)
        d.text((cx+42, y+28), step, font=F['small_bold'], fill='white')

        # Drake image
        paste_img(canvas, DRAKE_IMAGES[key],
                  (cx+30, y+90, cx+cw-30, y+90+img_h), border_color=color, bw=5)

        # Title
        d = ImageDraw.Draw(canvas)
        d.text((cx+30, y+90+img_h+20), f'{step}  {title}', font=F['label'], fill=NAVY)

        # Body
        d.multiline_text((cx+30, y+90+img_h+70), body,
                         font=F['body'], fill=MUTED, spacing=8)

    # Arrows
    d = ImageDraw.Draw(canvas)
    for i in range(2):
        ax = M + (i+1)*(cw+40) - 28
        d.text((ax, y + 90 + img_h//2 - 30), '\u2192', font=F['arrow'], fill='#a0a8b0')

    y += ch + 60

    # ══════════════════════════════════════════════════════════════════
    # SECTION 2: Why it stays stable
    # ══════════════════════════════════════════════════════════════════
    d = ImageDraw.Draw(canvas)
    d.text((M, y), '2   Why the system stays stable', font=F['section'], fill=NAVY)
    y += 80

    # Left: theory card
    theory_w = int(CW * 0.42)
    theory_h = 870
    card(canvas, (M, y, M+theory_w, y+theory_h), fill=OFFWHITE)
    d = ImageDraw.Draw(canvas)

    ty = y + 30
    d.text((M+35, ty), 'Topology-invariant error dynamics', font=F['label'], fill=NAVY)
    ty += 55

    dwrap(d, (M+35, ty),
          'Cable snaps change the disturbance input d_i(t), '
          'but not the closed-loop matrices (A, B). '
          'The error dynamics remain stable across all cable topologies.',
          F['body'], MUTED, theory_w-70, spacing=8)
    ty += 150

    # Equation box
    d.rounded_rectangle((M+40, ty, M+theory_w-40, ty+110), 24,
                         fill=LIGHT_BLUE, outline='#c0d0e0', width=3)
    eq = 'z_dot = A z + B d_i(t)'
    eb = d.textbbox((0,0), eq, font=F['equation'])
    ex = M + theory_w//2 - (eb[2]-eb[0])//2
    d.text((ex, ty+22), eq, font=F['equation'], fill=NAVY)
    ty += 145

    # Bullets
    bullets = [
        ('Proposition 1',
         'A, B depend only on PID gains (kp, kd, ki) \u2014\n'
         'independent of N, payload mass, or cable topology.'),
        ('Common Lyapunov',
         'A single Lyapunov function works across all\n'
         'cable topologies \u2014 no switching needed.'),
        ('Design tools',
         'Cost ratio: \u03c1_FT = N/(N\u2212k).\n'
         'Thrust margin: f_max > (Nm_Q+m_L)g/(N\u2212k) + f_track.'),
        ('Oracle comparison',
         'A centralized controller with perfect fault\n'
         'knowledge improves RMSE by only 0.7%.'),
    ]
    for head, body in bullets:
        d.ellipse((M+40, ty+10, M+60, ty+30), fill=GOLD)
        d.text((M+75, ty), head, font=F['small_bold'], fill=NAVY)
        d.multiline_text((M+75, ty+40), body, font=F['small'], fill=MUTED, spacing=6)
        ty += 120

    # Metric pills
    pill(canvas, M+40, ty+10, '<3% MC variation', GOLD, height=54)
    pill(canvas, M+350, ty+10, '0.7% oracle gain', GOLD, height=54)

    # Right: tracking error plot
    plot_x = M + theory_w + 50
    plot_w = CW - theory_w - 50
    plot_h = theory_h - 60

    card(canvas, (plot_x, y, plot_x+plot_w, y+theory_h), fill=OFFWHITE)
    d = ImageDraw.Draw(canvas)
    d.text((plot_x+30, y+25),
           'Tracking error stays bounded after each fault',
           font=F['label'], fill=NAVY)

    err_img = mpl_panel(plot_error, plot_w-60, plot_h-80)
    canvas.alpha_composite(err_img, (plot_x+30, y+80))

    y += theory_h + 60

    # ══════════════════════════════════════════════════════════════════
    # SECTION 3: Evidence
    # ══════════════════════════════════════════════════════════════════
    d = ImageDraw.Draw(canvas)
    d.text((M, y), '3   Evidence from the multibody simulations', font=F['section'], fill=NAVY)
    y += 80

    ev_cw = (CW - 80) // 3
    ev_ch = 600
    ev_plot_h = ev_ch - 100

    evidence = [
        ('Cable tensions redistribute\nafter each snap', plot_tensions),
        ('Payload path remains close\nto the reference', plot_xy),
        ('Larger teams dilute each\nfault\'s impact', plot_rmse),
    ]

    for i, (title, fn) in enumerate(evidence):
        cx = M + i*(ev_cw+40)
        card(canvas, (cx, y, cx+ev_cw, y+ev_ch), fill=OFFWHITE)
        d = ImageDraw.Draw(canvas)
        d.multiline_text((cx+ev_cw//2, y+20), title,
                         font=F['label'], fill=NAVY, anchor='ma', align='center', spacing=6)

        pimg = mpl_panel(fn, ev_cw-50, ev_plot_h-30)
        canvas.alpha_composite(pimg, (cx+25, y+95))

    y += ev_ch + 50

    # ══════════════════════════════════════════════════════════════════
    # FOOTER
    # ══════════════════════════════════════════════════════════════════
    d = ImageDraw.Draw(canvas)
    d.rounded_rectangle((M-20, y, W-M+20, y+100), 24, fill=NAVY)
    dwrap(d, (W//2, y+18),
          'Key takeaway: a local N-agnostic PID + gravity-feedforward controller '
          'can tolerate cascading cable snaps without communication '
          'or explicit fault detection.',
          F['small'], 'white', CW-100, align='center')

    # ── Save ──
    final = canvas.convert('RGB')
    final.save(OUT_PNG, dpi=(300,300))
    final.save(OUT_PDF, 'PDF', resolution=300.0)
    print(f'Saved {OUT_PNG}')
    print(f'Saved {OUT_PDF}')


if __name__ == '__main__':
    build()
