#!/usr/bin/env python3
"""
6D IMU live visualizer with fall detection overlay.

Reads serial lines of the form:
    BIAS,accel,<ax>,<ay>,<az>,gyro,<gx>,<gy>,<gz>
    DATA,<timestamp>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
    FALL,<type>,<hw_timestamp>,<peak_g>

as printed by the nRF52833 + ISM330DHCX firmware over USB CDC-ACM
(/dev/ttyACM0).

  FALL type values:
    DETECTED    - free-fall + impact confirmed
    WITH_INACT  - fall + impact + person lying still (more severe)
    CANCELLED   - free-fall observed but no impact (logged, not alarmed)

Left side : two scrolling line plots (accel x/y/z, gyro x/y/z vs time)
            Fall events are drawn as vertical marker lines on the accel
            plot: orange = DETECTED, red = WITH_INACT.
Right side: 3D orientation cube with complementary filter orientation.

Usage:
    python3 plot.py
    python3 plot.py --port /dev/ttyACM0 --baud 1000000
"""

import argparse
import collections
import sys
import time

import numpy as np
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# --------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1000000           # matches app.overlay current-speed
HISTORY_LEN  = 200               # samples kept on screen for line plots
ACCEL_YLIM   = (-2.5, 2.5)       # g
GYRO_YLIM    = (-300.0, 300.0)   # deg/s

# Complementary filter tuning for the 3D cube orientation
COMP_ALPHA = 0.96                # weight on gyro-integrated angle vs accel angle

# How long to show a fall alert banner on screen before clearing (seconds)
FALL_ALERT_DISPLAY_SECONDS = 5.0


def parse_args():
    p = argparse.ArgumentParser(description="6D IMU live visualizer with fall detection")
    p.add_argument("--port", default=DEFAULT_PORT, help="Serial port (default: %(default)s)")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate (default: %(default)s)")
    return p.parse_args()


def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as exc:
        print(f"Failed to open {port}: {exc}", file=sys.stderr)
        print("Check the port name (ls /dev/ttyACM*) and that nothing else "
              "has it open (close any serial terminal / minicom session).",
              file=sys.stderr)
        sys.exit(1)
    # Let the board finish any boot-time USB enumeration chatter
    time.sleep(0.3)
    ser.reset_input_buffer()
    return ser


def parse_line(line, bias):
    """Parse one serial line.

    Returns one of:
        ('data',      dict with ts/ax/ay/az/gx/gy/gz)
        ('bias',      dict with ax/ay/az/gx/gy/gz)
        ('fall',      dict with fall_type/hw_ts/peak_g)
        None  – unrecognised or malformed line
    """
    line = line.strip()
    if not line:
        return None

    parts = line.split(",")

    if parts[0] == "BIAS" and len(parts) >= 8:
        # BIAS,accel,ax,ay,az,gyro,gx,gy,gz
        try:
            ax, ay, az = float(parts[2]), float(parts[3]), float(parts[4])
            gx, gy, gz = float(parts[6]), float(parts[7]), float(parts[8] if len(parts) > 8 else parts[7])
        except (ValueError, IndexError):
            return None
        return ("bias", dict(ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz))

    if parts[0] == "DATA" and len(parts) == 8:
        # DATA,timestamp,ax,ay,az,gx,gy,gz
        try:
            ts = int(parts[1])
            ax, ay, az = float(parts[2]), float(parts[3]), float(parts[4])
            gx, gy, gz = float(parts[5]), float(parts[6]), float(parts[7])
        except ValueError:
            return None
        return ("data", dict(ts=ts, ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz))

    if parts[0] == "FALL" and len(parts) == 4:
        # FALL,<type>,<hw_timestamp>,<peak_g>
        try:
            fall_type = parts[1]          # DETECTED / WITH_INACT / CANCELLED
            hw_ts     = int(parts[2])
            peak_g    = float(parts[3])
        except (ValueError, IndexError):
            return None
        return ("fall", dict(fall_type=fall_type, hw_ts=hw_ts, peak_g=peak_g))

    return None


# --------------------------------------------------------------------
# 3D cube geometry helpers (unchanged from original)
# --------------------------------------------------------------------
def cube_vertices(size=1.0):
    s = size / 2.0
    return np.array([
        [-s, -s, -s], [s, -s, -s], [s, s, -s], [-s, s, -s],
        [-s, -s,  s], [s, -s,  s], [s, s,  s], [-s, s,  s],
    ])


CUBE_FACE_IDX = [
    [0, 1, 2, 3], [4, 5, 6, 7],
    [0, 1, 5, 4], [2, 3, 7, 6],
    [1, 2, 6, 5], [0, 3, 7, 4],
]


def rotation_matrix(roll, pitch, yaw):
    """Build a body->world rotation matrix from roll/pitch/yaw (radians)."""
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    rx = np.array([[1,  0,   0 ],
                   [0,  cr, -sr],
                   [0,  sr,  cr]])
    ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    return rz @ ry @ rx


class OrientationEstimator:
    """Simple complementary filter: accel gives a noisy but drift-free
    roll/pitch reference, gyro integration gives smooth short-term
    motion (and is the only source for yaw, which accel cannot sense)."""

    def __init__(self):
        self.roll   = 0.0
        self.pitch  = 0.0
        self.yaw    = 0.0
        self.last_t = None

    def update(self, ax, ay, az, gx_dps, gy_dps, gz_dps, now):
        if self.last_t is None:
            dt = 0.0
        else:
            dt = max(0.0, min(now - self.last_t, 0.5))  # clamp for safety
        self.last_t = now

        accel_roll  = np.arctan2(ay, az)
        accel_pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az))

        gx = np.radians(gx_dps)
        gy = np.radians(gy_dps)
        gz = np.radians(gz_dps)

        gyro_roll  = self.roll  + gx * dt
        gyro_pitch = self.pitch + gy * dt
        gyro_yaw   = self.yaw   + gz * dt

        self.roll  = COMP_ALPHA * gyro_roll  + (1.0 - COMP_ALPHA) * accel_roll
        self.pitch = COMP_ALPHA * gyro_pitch + (1.0 - COMP_ALPHA) * accel_pitch
        self.yaw   = gyro_yaw

        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))

        return self.roll, self.pitch, self.yaw


# --------------------------------------------------------------------
# Main
# --------------------------------------------------------------------
def main():
    args = parse_args()
    ser  = open_serial(args.port, args.baud)
    print(f"Listening on {args.port} @ {args.baud} baud. Ctrl+C to quit.")

    bias      = dict(ax=0.0, ay=0.0, az=1.0, gx=0.0, gy=0.0, gz=0.0)
    estimator = OrientationEstimator()

    # Rolling history buffers for line plots
    t_hist  = collections.deque(maxlen=HISTORY_LEN)
    ax_hist = collections.deque(maxlen=HISTORY_LEN)
    ay_hist = collections.deque(maxlen=HISTORY_LEN)
    az_hist = collections.deque(maxlen=HISTORY_LEN)
    gx_hist = collections.deque(maxlen=HISTORY_LEN)
    gy_hist = collections.deque(maxlen=HISTORY_LEN)
    gz_hist = collections.deque(maxlen=HISTORY_LEN)

    # Fall event log: list of (wall_time, fall_type, peak_g)
    fall_events = []
    # Active alert banner: (expiry_wall_time, message_string, color)
    fall_alert = [None]

    # Fall counter for title
    fall_count = [0]

    t0 = time.time()

    # ---------------- Figure layout ----------------
    fig = plt.figure(figsize=(14, 7))
    fig.suptitle("ISM330DHCX 6D Live Visualizer  |  Fall Detection Active",
                 fontsize=11, fontweight="bold")

    ax_accel = fig.add_subplot(2, 2, 1)
    ax_gyro  = fig.add_subplot(2, 2, 3)
    ax_3d    = fig.add_subplot(1, 2, 2, projection="3d")

    (line_ax,) = ax_accel.plot([], [], label="ax (g)",  color="tab:red")
    (line_ay,) = ax_accel.plot([], [], label="ay (g)",  color="tab:green")
    (line_az,) = ax_accel.plot([], [], label="az (g)",  color="tab:blue")
    ax_accel.set_ylim(*ACCEL_YLIM)
    ax_accel.set_ylabel("Accel (g)")
    ax_accel.legend(loc="upper right", fontsize=8)
    ax_accel.grid(True, alpha=0.3)
    ax_accel.set_title("Accelerometer  (fall events = vertical lines)")

    (line_gx,) = ax_gyro.plot([], [], label="gx (dps)", color="tab:red")
    (line_gy,) = ax_gyro.plot([], [], label="gy (dps)", color="tab:green")
    (line_gz,) = ax_gyro.plot([], [], label="gz (dps)", color="tab:blue")
    ax_gyro.set_ylim(*GYRO_YLIM)
    ax_gyro.set_xlabel("Time (s)")
    ax_gyro.set_ylabel("Gyro (deg/s)")
    ax_gyro.legend(loc="upper right", fontsize=8)
    ax_gyro.grid(True, alpha=0.3)

    ax_3d.set_xlim(-1, 1)
    ax_3d.set_ylim(-1, 1)
    ax_3d.set_zlim(-1, 1)
    ax_3d.set_xlabel("X")
    ax_3d.set_ylabel("Y")
    ax_3d.set_zlabel("Z")
    ax_3d.set_title("Orientation")

    verts  = cube_vertices(1.0)
    faces  = [[verts[i] for i in face] for face in CUBE_FACE_IDX]
    cube_poly = Poly3DCollection(faces, facecolors="tab:blue",
                                 edgecolors="k", alpha=0.7)
    ax_3d.add_collection3d(cube_poly)

    status_text = fig.text(0.5, 0.02, "Waiting for data...",
                           ha="center", fontsize=9)
    # Fall alert banner (large, centred, overlaid on the figure)
    alert_text = fig.text(0.5, 0.5, "",
                          ha="center", va="center", fontsize=22,
                          fontweight="bold", color="white",
                          bbox=dict(boxstyle="round,pad=0.5",
                                    facecolor="red", alpha=0.0),
                          zorder=10)

    line_buffer = ""

    def read_available_lines():
        nonlocal line_buffer
        try:
            n = ser.in_waiting
        except OSError:
            return []
        if n == 0:
            return []
        chunk = ser.read(n).decode("utf-8", errors="replace")
        line_buffer += chunk
        lines = line_buffer.split("\n")
        line_buffer = lines[-1]
        return [l for l in lines[:-1] if l.strip()]

    # Keep references to fall vlines so we can prune them when they
    # scroll out of the visible window
    fall_vlines = []  # list of (wall_time, axvline_handle)

    def add_fall_vline(now_wall, fall_type):
        """Draw a vertical line on the accel plot at current time."""
        color = "red" if fall_type == "WITH_INACT" else "orange"
        label = f"{'⚠ FALL+INACT' if fall_type == 'WITH_INACT' else '⚠ FALL'}"
        vl = ax_accel.axvline(x=(now_wall - t0), color=color,
                              linewidth=2.0, linestyle="--",
                              label=label, alpha=0.8)
        fall_vlines.append((now_wall, vl))

    def show_fall_alert(fall_type, peak_g):
        if fall_type == "WITH_INACT":
            msg  = f"⚠  FALL DETECTED  ⚠\nPeak impact: {peak_g:.2f} g\nPerson may be lying still!"
            color = "darkred"
        else:
            msg  = f"⚠  FALL DETECTED  ⚠\nPeak impact: {peak_g:.2f} g"
            color = "darkorange"

        expiry = time.time() + FALL_ALERT_DISPLAY_SECONDS
        fall_alert[0] = (expiry, msg, color)

        alert_text.set_text(msg)
        alert_text.get_bbox_patch().set_facecolor(color)
        alert_text.get_bbox_patch().set_alpha(0.85)
        fig.canvas.draw_idle()

    def update(_frame):
        nonlocal fall_vlines
        new_data = False
        now_wall = time.time()

        for raw_line in read_available_lines():
            parsed = parse_line(raw_line, bias)
            if parsed is None:
                # Print unknown lines so they show in the terminal
                if raw_line.strip():
                    print(f"[serial] {raw_line.strip()}")
                continue

            kind, payload = parsed

            if kind == "bias":
                bias.update(payload)
                status_text.set_text(
                    f"Bias captured: accel=({bias['ax']:.3f},{bias['ay']:.3f},{bias['az']:.3f}) g"
                )
                continue

            if kind == "fall":
                ft     = payload["fall_type"]
                peak_g = payload["peak_g"]
                hw_ts  = payload["hw_ts"]

                print(f"[FALL EVENT] type={ft}  peak={peak_g:.3f} g  hw_ts={hw_ts}")

                if ft in ("DETECTED", "WITH_INACT"):
                    fall_count[0] += 1
                    fall_events.append((now_wall, ft, peak_g))
                    add_fall_vline(now_wall, ft)
                    show_fall_alert(ft, peak_g)
                continue

            if kind == "data":
                t_hist.append(now_wall - t0)
                ax_hist.append(payload["ax"])
                ay_hist.append(payload["ay"])
                az_hist.append(payload["az"])
                gx_hist.append(payload["gx"])
                gy_hist.append(payload["gy"])
                gz_hist.append(payload["gz"])

                estimator.update(
                    payload["ax"], payload["ay"], payload["az"],
                    payload["gx"], payload["gy"], payload["gz"],
                    now_wall,
                )
                new_data = True

        # --- Expire alert banner ---
        if fall_alert[0] is not None:
            expiry, _msg, _color = fall_alert[0]
            if now_wall > expiry:
                alert_text.set_text("")
                alert_text.get_bbox_patch().set_alpha(0.0)
                fall_alert[0] = None

        if not new_data:
            return (line_ax, line_ay, line_az,
                    line_gx, line_gy, line_gz, cube_poly)

        t_arr = list(t_hist)

        line_ax.set_data(t_arr, list(ax_hist))
        line_ay.set_data(t_arr, list(ay_hist))
        line_az.set_data(t_arr, list(az_hist))
        line_gx.set_data(t_arr, list(gx_hist))
        line_gy.set_data(t_arr, list(gy_hist))
        line_gz.set_data(t_arr, list(gz_hist))

        if t_arr:
            xmin = max(0, t_arr[0])
            xmax = max(t_arr[-1], t_arr[0] + 1)
            ax_accel.set_xlim(xmin, xmax)
            ax_gyro.set_xlim(xmin, xmax)

            # Remove vlines that have scrolled off the left edge
            fall_vlines = [(wt, vl) for wt, vl in fall_vlines
                           if (wt - t0) >= xmin]

        # Rotate cube
        rot = rotation_matrix(estimator.roll, estimator.pitch, estimator.yaw)
        rotated   = verts @ rot.T
        new_faces = [[rotated[i] for i in face] for face in CUBE_FACE_IDX]
        cube_poly.set_verts(new_faces)

        fall_str = f"  |  Falls detected: {fall_count[0]}" if fall_count[0] else ""
        status_text.set_text(
            f"roll={np.degrees(estimator.roll):6.1f}  "
            f"pitch={np.degrees(estimator.pitch):6.1f}  "
            f"yaw={np.degrees(estimator.yaw):6.1f}  (deg)   "
            f"samples={len(t_hist)}"
            f"{fall_str}"
        )

        return (line_ax, line_ay, line_az,
                line_gx, line_gy, line_gz, cube_poly)

    ani = animation.FuncAnimation(fig, update, interval=30, blit=False)

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if fall_count[0]:
            print(f"\n--- Session Summary ---")
            print(f"Total falls detected: {fall_count[0]}")
            for i, (wt, ft, pg) in enumerate(fall_events, 1):
                ts_str = time.strftime("%H:%M:%S", time.localtime(wt))
                print(f"  [{i}] {ts_str}  type={ft}  peak={pg:.3f} g")


if __name__ == "__main__":
    main()
