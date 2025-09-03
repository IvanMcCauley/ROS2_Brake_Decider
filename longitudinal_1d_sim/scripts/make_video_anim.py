#!/usr/bin/env python3
"""
make_video_anim.py — CSV→MP4 replay for longitudinal_1d_sim.

Reads the newest CSV (t,x,v,d,brake_cmd,u) from ~/ros2_ws/logs and renders:
  • Left: car vs wall + brake light (red when brake_cmd==1)
  • Right: v(t) and d(t) with shaded braking intervals + moving time cursor + live HUD
Saves MP4 to ~/ros2_ws/results/videos/. This is a replay of your real closed-loop run.
"""
import os, glob, csv
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

# ---------- paths ----------
LOG_DIR = Path.home() / 'ros2_ws' / 'logs'
OUT_DIR = Path.home() / 'ros2_ws' / 'results' / 'videos'
OUT_DIR.mkdir(parents=True, exist_ok=True)

paths = sorted(glob.glob(str(LOG_DIR / 'longitudinal_sim_*.csv')))
if not paths:
    raise SystemExit(f'No CSV files in {LOG_DIR}')
csv_path = paths[-1]

# ---------- load CSV ----------
t, x, v, d, b, u = [], [], [], [], [], []
with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row['t']))
        x.append(float(row['x']))
        v.append(float(row['v']))
        d.append(float(row['d']))
        b.append(int(row['brake_cmd']))
        u.append(float(row.get('u', '0.0')))  # older logs may not have 'u'

t = np.asarray(t); x = np.asarray(x); v = np.asarray(v); d = np.asarray(d)
b = np.asarray(b, dtype=int); u = np.asarray(u)

# ---------- derived constants ----------
x_obs = x[0] + d[0]                 # obstacle position at start
car_len, car_h = 4.0, 1.6           # purely visual
dt = float(t[1] - t[0]) if len(t) > 1 else 0.05
v0 = float(v[0]); d0 = float(d[0]); decel = max(1e-6, float(np.abs(u).max()))  # ≈9 m/s²

RT = 1.0        # reaction_time used
MARGIN = 0.10   # safety_margin used

# ---------- figure/text layout ----------
fig = plt.figure(figsize=(10, 5), constrained_layout=True)
fig.suptitle('ROS 2 closed-loop replay: sim ↔ brake_decider', fontsize=12, fontweight='bold')
fig.text(0.02, 0.10,
    f"Topics: /ego_speed   /obstacle_distance   /brake_cmd\n"
    f"Params: rt={RT:.2f}s    safety margin={MARGIN:.2f}   decel≈{decel:.1f} m/s^2", fontsize=8)
fig.text(0.02, 0.2,'Decision: BRAKE if d_stop ≥ d_avail\n Where: d_stop = v*rt + v²/(2a)\n             d_avail = d*(1−m)', fontsize=10)

gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 1.0], height_ratios=[1, 1])
ax_world = fig.add_subplot(gs[:, 0])
ax_v     = fig.add_subplot(gs[0, 1])
ax_d     = fig.add_subplot(gs[1, 1])

# ---------- left: world view ----------
ax_world.set_title('World view')
ax_world.set_aspect('equal', adjustable='box')
xmin = min(0.0, float(x.min()) - 5.0)
xmax = float(x_obs) + 10.0
ax_world.set_xlim(xmin, xmax)
ax_world.set_ylim(-2, 4)

# road baseline (neutral gray) and wall
ax_world.hlines(0.0, xmin, xmax, linewidth=0.5, color='0.6', alpha=0.6)
ax_world.plot([x_obs, x_obs], [-2, 4], linewidth=5, color='0.2')

# car body
car = plt.Rectangle((x[0] - car_len, 0.0), car_len, car_h, fill=True, color='0.2')
# rear brake light: full car width, centered vertically
side   = car_h * 0.8        # brake light square size
margin = -1          # gap from the very rear edge
light_y = (car_h - side) / 2  # center vertically on the car
light = plt.Rectangle((x[0] - car_len + margin, light_y),
                      side, side,
                      fill=True, facecolor='red',
                      zorder=6, visible=bool(b[0]))
ax_world.add_patch(car); ax_world.add_patch(light)

# hide world-view Y axis (keep it clean)
ax_world.get_yaxis().set_visible(False)
ax_world.set_xlabel('x [m]')

# Live HUD above the world view
hud = ax_world.text(0.02, 2.0, "", transform=ax_world.transAxes,
                    fontsize=10, va='bottom', clip_on=False)

# ---------- right: plots ----------
ax_v.set_title('Velocity v(t)'); ax_v.plot(t, v)
ax_v.set_xlabel('t [s]'); ax_v.set_ylabel('v [m/s]')

ax_d.set_title('Distance d(t)'); ax_d.plot(t, d, linestyle='--')
ax_d.set_xlabel('t [s]'); ax_d.set_ylabel('d [m]')

# time cursors (move during animation)
v_cursor = ax_v.axvline(t[0], linewidth=1, color='0.2')
d_cursor = ax_d.axvline(t[0], linewidth=1, color='0.2')

# ---------- animation ----------
def update(i: int):
    # move car + brake light color
    car.set_xy((x[i] - car_len, 0.0))
    # brake light: show only when braking
    light.set_xy((x[i] - car_len + margin, light_y))
    light.set_visible(bool(b[i]))

    # move time cursors
    v_cursor.set_xdata([t[i], t[i]])
    d_cursor.set_xdata([t[i], t[i]])

    # HUD numbers
    hud.set_text(f"t={t[i]:.2f}s     v={v[i]:.1f} m/s     d={d[i]:.1f} m     Brake:{'ON' if b[i] else 'OFF'}")

    return car, light, v_cursor, d_cursor

ani = animation.FuncAnimation(fig, update, frames=len(t),
                              interval=max(1, int(1000 * dt)), blit=False)

stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
mp4_path = OUT_DIR / f'sim_{stamp}.mp4'

# ---------- save (prefer MP4, fallback to GIF) ----------
try:
    writer = animation.FFMpegWriter(fps=int(round(1.0 / dt)) if dt > 0 else 20, bitrate=2000)
    ani.save(str(mp4_path), writer=writer, dpi=160)
    print(f'Wrote video: {mp4_path}')
except Exception:
    from matplotlib.animation import PillowWriter
    gif_path = OUT_DIR / f'sim_{stamp}.gif'
    ani.save(str(gif_path), writer=PillowWriter(fps=20))
    print(f'ffmpeg not available, wrote GIF instead: {gif_path}')
