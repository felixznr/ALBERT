#!/usr/bin/env python3
"""
ALBERT Flight Computer — Serial Monitor & Attitude Visualiser

Frame format (from firmware):
    $ALB,<ax>,<ay>,<az>,<u>,<v>,<w>,<phi_deg>,<theta_deg>,<psi_deg>\r\n
"""

import argparse
import collections
import threading
import time
import sys

import numpy as np
import matplotlib
matplotlib.rcParams['toolbar'] = 'None'
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import serial
import serial.tools.list_ports

BAUD    = 115200
HISTORY = 300

BG      = "#0d1117"
BG_FIG  = "#161b22"
GREY    = "#8b949e"
WHITE   = "#e6edf3"


# ── data store ─────────────────────────────────────────────────────────────────

class DataStore:
    _KEYS = ("t", "ax", "ay", "az", "u", "v", "w", "phi", "theta", "psi")

    def __init__(self, n=HISTORY):
        self._lock  = threading.Lock()
        self._buf   = {k: collections.deque([0.0] * n, maxlen=n) for k in self._KEYS}
        self._t0    = None
        self.latest = {k: 0.0 for k in self._KEYS}
        self.frame_count = 0

    def push(self, ax, ay, az, u, v, w, phi, theta, psi):
        now = time.monotonic()
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            vals = dict(t=now - self._t0,
                        ax=ax, ay=ay, az=az,
                        u=u,   v=v,   w=w,
                        phi=phi, theta=theta, psi=psi)
            for k, val in vals.items():
                self._buf[k].append(val)
            self.latest = {k: self._buf[k][-1] for k in self._KEYS}
            self.frame_count += 1

    def snapshot(self):
        with self._lock:
            return {k: np.array(self._buf[k]) for k in self._KEYS}


# ── serial reader ───────────────────────────────────────────────────────────────

def serial_reader(port: str, store: DataStore, stop: threading.Event):
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
        print(f"[serial] opened {port} @ {BAUD}")
    except serial.SerialException as e:
        print(f"[serial] ERROR: {e}")
        stop.set()
        return

    while not stop.is_set():
        try:
            line = ser.readline()
            if not line.startswith(b"$ALB,"):
                continue
            parts = line.strip().split(b",")
            if len(parts) != 10:
                continue
            store.push(*[float(p) for p in parts[1:]])
        except (ValueError, serial.SerialException):
            pass

    ser.close()
    print("[serial] closed")


# ── math ────────────────────────────────────────────────────────────────────────

def R_nb(phi_deg, theta_deg, psi_deg):
    """Nav←body: columns are body axes expressed in nav frame (3-2-1 Euler)."""
    phi   = np.radians(phi_deg)
    theta = np.radians(theta_deg)
    psi   = np.radians(psi_deg)
    cp, sp = np.cos(phi),   np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cy, sy = np.cos(psi),   np.sin(psi)
    Rbn = np.array([
        [ ct*cy,           ct*sy,          -st    ],
        [-cp*sy+sp*st*cy,  cp*cy+sp*st*sy,  sp*ct ],
        [ sp*sy+cp*st*cy, -sp*cy+cp*st*sy,  cp*ct ],
    ])
    return Rbn.T


# ── draw functions ──────────────────────────────────────────────────────────────

def nav_to_plot(v):
    """Remap nav frame [nx, ny, nz] → matplotlib 3D [x, y, z].
    nav-X (up) → plot-Z (vertical), nav-Y → plot-X, nav-Z → plot-Y."""
    return np.array([v[1], v[2], v[0]])


def draw_3d(ax, phi, theta, psi):
    ax.cla()
    ax.set_facecolor(BG)
    lim = 1.4
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_xlabel("N-Y",      color=GREY, fontsize=8, labelpad=2)
    ax.set_ylabel("N-Z",      color=GREY, fontsize=8, labelpad=2)
    ax.set_zlabel("N-X (up)", color=GREY, fontsize=8, labelpad=2)
    ax.tick_params(colors=GREY, labelsize=7)
    ax.set_title(f"φ = {phi:.1f}°     θ = {theta:.1f}°     ψ = {psi:.1f}°",
                 color=WHITE, fontsize=11, pad=6)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("#30363d")
    ax.yaxis.pane.set_edgecolor("#30363d")
    ax.zaxis.pane.set_edgecolor("#30363d")
    ax.grid(color="#30363d", linewidth=0.5)

    Rnb = R_nb(phi, theta, psi)
    bx, by, bz = Rnb[:, 0], Rnb[:, 1], Rnb[:, 2]

    tail, nose = nav_to_plot(-0.75 * bx), nav_to_plot(0.75 * bx)
    ax.plot([tail[0], nose[0]], [tail[1], nose[1]], [tail[2], nose[2]],
            color=WHITE, lw=4)
    ax.scatter(*nose, color="orange", s=120, zorder=6)

    O = np.zeros(3)
    for vec, col in zip([bx, by, bz], ["#ff6b6b", "#69db7c", "#74c0fc"]):
        pv = nav_to_plot(vec)
        ax.quiver(*O, *pv, color=col, length=1.0,
                  arrow_length_ratio=0.15, linewidth=2.0)

    # Faint nav reference axes (X=up, Y, Z)
    for nav_i, col in enumerate(["#ff000040", "#00ff0040", "#0000ff40"]):
        v = np.zeros(3); v[nav_i] = 1.0
        pv = nav_to_plot(v)
        ax.quiver(*O, *pv, color=col, length=0.45,
                  arrow_length_ratio=0.2, linewidth=0.8, linestyle=":")


def draw_timeseries(ax, t, series: dict, title, ylabel):
    ax.cla()
    ax.set_facecolor(BG)
    ax.set_title(title, color=WHITE, fontsize=11, pad=4)
    ax.set_ylabel(ylabel, color=GREY, fontsize=9)
    ax.set_xlabel("t [s]", color=GREY, fontsize=9)
    ax.tick_params(colors=GREY, labelsize=8)
    for (lbl, arr), col in zip(series.items(),
                                ["#ff6b6b", "#69db7c", "#74c0fc"]):
        ax.plot(t, arr, color=col, lw=1.5, label=lbl)
    ax.legend(loc="upper left", fontsize=9,
              framealpha=0.2, labelcolor=WHITE,
              facecolor=BG, edgecolor="#30363d")
    for sp in ax.spines.values():
        sp.set_color("#30363d")
    if len(t) > 1:
        ax.set_xlim(t[0], max(t[-1], t[0] + 1))


# ── figure ──────────────────────────────────────────────────────────────────────

def build_figure():
    fig = plt.figure(figsize=(15, 8), facecolor=BG_FIG)
    fig.suptitle("ALBERT  —  Flight Computer Monitor",
                 color=WHITE, fontsize=14, fontweight="bold", y=0.98)

    gs = fig.add_gridspec(2, 2, left=0.04, right=0.97,
                          top=0.93, bottom=0.07,
                          hspace=0.38, wspace=0.28)

    ax3d  = fig.add_subplot(gs[:, 0], projection="3d")
    ax_vel = fig.add_subplot(gs[0, 1])
    ax_acc = fig.add_subplot(gs[1, 1])

    for ax in (ax_vel, ax_acc):
        ax.set_facecolor(BG)
        for sp in ax.spines.values():
            sp.set_color("#30363d")

    ax3d.set_facecolor(BG)
    return fig, ax3d, ax_vel, ax_acc


# ── main ────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ALBERT serial visualiser")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}   {p.description}")
        sys.exit(0)

    if args.port is None:
        args.port = "/dev/tty.usbmodem3857345533351"
        print(f"[default] using {args.port}")

    store  = DataStore()
    stop   = threading.Event()
    reader = threading.Thread(target=serial_reader,
                              args=(args.port, store, stop), daemon=True)
    reader.start()

    fig, ax3d, ax_vel, ax_acc = build_figure()

    status_text = fig.text(0.50, 0.005, "waiting for frames…",
                           ha="center", va="bottom", color=GREY, fontsize=8)

    def update(_frame):
        d   = store.snapshot()
        lat = store.latest

        draw_3d(ax3d, lat["phi"], lat["theta"], lat["psi"])
        draw_timeseries(ax_vel, d["t"],
                        {"u": d["u"], "v": d["v"], "w": d["w"]},
                        "Velocity  [m/s]", "m/s")
        draw_timeseries(ax_acc, d["t"],
                        {"ax": d["ax"], "ay": d["ay"], "az": d["az"]},
                        "Accel  [m/s²]", "m/s²")

        status_text.set_text(
            f"frames: {store.frame_count}   "
            f"ax={lat['ax']:+.2f}  ay={lat['ay']:+.2f}  az={lat['az']:+.2f} m/s²   "
            f"u={lat['u']:+.2f} m/s")

    ani = animation.FuncAnimation(fig, update, interval=100,
                                  cache_frame_data=False)

    try:
        plt.show()
    finally:
        stop.set()


if __name__ == "__main__":
    main()