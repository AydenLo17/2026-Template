#!/usr/bin/env python3
"""
scripts/gamepiece_vision_test.py
=================================
Visualizes and validates the camera math inside GamepieceAssistDrive -- no real robot needed.

HOW IT WORKS
  Forward model:  robot pose + known gamepiece field position -> synthetic tx / ty
                  (exactly what a perfect Limelight would report)

  Inverse model:  tx / ty + robot pose -> estimated field position
                  (identical trig to GamepieceAssistDrive.estimateGamepieceRobotRelative)

  The demo animates a robot sweeping past gamepieces, shows the camera FOV cone,
  overlays true vs. estimated positions, and draws the assist vector.
  Estimation error is shown live so you can see how well the math closes the loop.

REQUIREMENTS
  pip install matplotlib numpy
  pip install robotpy-ntcore   # optional, only needed for --nt

USAGE
  python scripts/gamepiece_vision_test.py                  # animated matplotlib demo
  python scripts/gamepiece_vision_test.py --nt             # + stream to AdvantageScope (NT4 port 5810)
  python scripts/gamepiece_vision_test.py --noise 1.5      # add Gaussian noise sigma=1.5 deg to tx/ty
  python scripts/gamepiece_vision_test.py --speed 3        # run 3x faster
  python scripts/gamepiece_vision_test.py --save demo.gif  # save animation to GIF

ADVANTAGESCOPE SETUP (when using --nt)
  1. Run:  python scripts/gamepiece_vision_test.py --nt
  2. Open AdvantageScope -> File -> Connect to NT4 -> localhost:5810
  3. Drag "SmartDashboard/Field" onto a 2D Field widget -> set to match year
  4. Drag scalar keys under SmartDashboard/VisionTest/ onto graphs / numeric displays

NT KEYS PUBLISHED
  SmartDashboard/Field/Robot             double[3]    robot pose  [x_m, y_m, rot_deg]
  SmartDashboard/Field/TrueGamepieces    double[3n]   actual gamepiece positions
  SmartDashboard/Field/EstimatedPieces   double[3n]   camera-math estimates
  SmartDashboard/Field/AssistArrow       double[6]    robot-to-target line (2 pose objects)
  SmartDashboard/Field/CameraFOV         double[9]    3-point FOV triangle (origin + 2 edges)
  SmartDashboard/VisionTest/TxDeg        double       current best-target tx (degrees)
  SmartDashboard/VisionTest/TyDeg        double       current best-target ty (degrees)
  SmartDashboard/VisionTest/RangeMeters  double       estimated range to best target
  SmartDashboard/VisionTest/ErrorMeters  double       estimation error vs ground truth
  SmartDashboard/VisionTest/HasTarget    boolean      camera currently has a valid detection
  SmartDashboard/VisionTest/InFOV        int          number of pieces currently in camera FOV
"""

import argparse
import math
import sys
import time
from typing import List, Optional, Tuple

import numpy as np

# ─── optional NT import ───────────────────────────────────────────────────────
try:
    import ntcore

    NT_AVAILABLE = True
except ImportError:
    NT_AVAILABLE = False

# ─── mirror of Constants.Gamepiece ────────────────────────────────────────────
CAM_PITCH_DEG = 24.0  # mount pitch – positive = camera looks down toward carpet (deg)
CAM_YAW_DEG = 0.0  # mount yaw offset – positive = camera points robot-left (deg)
CAM_HEIGHT_M = 0.56  # camera lens height above carpet (m)
CAM_FORWARD_M = 0.30  # camera forward offset from robot center (m)
CAM_LEFT_M = 0.00  # camera left offset from robot center (m)
PIECE_HEIGHT_M = 0.02  # gamepiece center height above carpet (m)
STOP_DIST_M = 0.22  # kStopDistanceMeters — inside this → no assist
PURSUIT_GAIN = 1.8  # kPursuitGain (1/s)
MAX_ASSIST_MPS = 1.8  # kMaxAssistSpeedMetersPerSecond

# Limelight 3G typical FOV limits
CAM_HFOV_DEG = 63.3
CAM_VFOV_DEG = 49.7
MAX_RANGE_M = 4.5  # approximate useful detection range (area-filter equivalent)

# Field dimensions (m)
FIELD_W = 16.54
FIELD_H = 8.21

# ─── demo scenario ────────────────────────────────────────────────────────────
# Six gamepieces placed at representative field positions.
GAMEPIECES = [
    (2.50, 1.80),
    (2.50, 4.10),
    (2.50, 6.30),
    (5.00, 2.70),
    (5.00, 5.50),
    (8.27, 4.10),
]

# Robot waypoints for the animated demo path (closed loop).
# Dense enough that the robot naturally sweeps through each gamepiece's vicinity.
RAW_WAYPOINTS = [
    (1.20, 4.10),
    (2.10, 2.20),
    (1.80, 4.10),
    (2.10, 6.00),
    (3.80, 6.50),
    (5.20, 5.80),
    (5.20, 2.40),
    (3.80, 1.50),
    (5.00, 4.10),
    (7.00, 3.00),
    (8.50, 2.00),
    (8.50, 4.10),
    (8.50, 6.20),
    (7.00, 5.20),
    (5.00, 4.10),
    (3.00, 4.10),
    (1.20, 4.10),
]

# Animation/loop timing
ANIM_MS = 20  # matplotlib animation frame interval (milliseconds, ~50 fps)
LOOP_HZ = 50  # NetworkTables update rate (frames per second)

# ─── forward model ────────────────────────────────────────────────────────────
# Given the robot's true field pose and a gamepiece's true field position, compute
# the tx / ty values that a perfect Limelight would report.


def forward_model(
    robot_x: float,
    robot_y: float,
    robot_heading_rad: float,
    piece_x: float,
    piece_y: float,
) -> Tuple[float, float, bool]:
    """
    Compute synthetic Limelight angles for a gamepiece at a known field position.

    Returns (tx_deg, ty_deg, in_fov).
    tx_deg: horizontal bearing, positive = right in camera frame
    ty_deg: vertical bearing from crosshair, negative = below (piece is on carpet)
    in_fov: False if the piece is outside the camera's field of view or too far away
    """
    # Camera position in field frame
    cam_x = robot_x + CAM_FORWARD_M * math.cos(robot_heading_rad) - CAM_LEFT_M * math.sin(robot_heading_rad)
    cam_y = robot_y + CAM_FORWARD_M * math.sin(robot_heading_rad) + CAM_LEFT_M * math.cos(robot_heading_rad)

    # Vector from camera to piece in field frame
    dx = piece_x - cam_x
    dy = piece_y - cam_y

    # Rotate into camera frame  (camera heading = robot heading + yaw offset)
    cam_heading = robot_heading_rad + math.radians(CAM_YAW_DEG)
    # forward = component along camera bore
    fwd = dx * math.cos(cam_heading) + dy * math.sin(cam_heading)
    # lateral = component to camera-left (positive = left = robotpy convention matches Java)
    lat = -dx * math.sin(cam_heading) + dy * math.cos(cam_heading)

    # Piece must be in front of the camera
    if fwd <= 0.05:
        return 0.0, 0.0, False

    horiz_dist = math.sqrt(fwd * fwd + lat * lat)

    # ── tx: horizontal bearing from camera bore ──
    # Positive tx means piece is to the right in camera view.
    # lat > 0 → piece is to the left in camera frame → tx < 0 (Limelight right-positive convention).
    # NOTE: the Java inverse uses  left = dist * sin(kCameraYaw + tx),
    #       which means positive bearing angle → positive left component.
    #       So tx here is in the "left-positive" robot bearing convention.
    #       We encode it consistently so forward/inverse round-trips to zero error.
    tx_rad = math.atan2(lat, fwd)  # bearing in camera frame, left-positive
    tx_deg = math.degrees(tx_rad) - CAM_YAW_DEG

    # ── ty: vertical angle from crosshair ──
    # totalPitchDeg = angle from horizontal to line-of-sight (positive = looking down)
    height_diff = CAM_HEIGHT_M - PIECE_HEIGHT_M  # positive: camera above piece
    total_pitch_deg = math.degrees(math.atan2(height_diff, horiz_dist))
    ty_deg = total_pitch_deg - CAM_PITCH_DEG  # negative when piece is below crosshair

    # ── FOV check ──
    half_hfov = CAM_HFOV_DEG / 2.0
    half_vfov = CAM_VFOV_DEG / 2.0
    in_fov = (
        abs(tx_deg) <= half_hfov
        and abs(ty_deg) <= half_vfov
        and horiz_dist <= MAX_RANGE_M
    )

    return tx_deg, ty_deg, in_fov


# ─── inverse model ─────────────────────────────────────────────────────────────
# Exact Java port of GamepieceAssistDrive.estimateGamepieceRobotRelative()
# followed by the field-frame rotation in updateGamepieceEstimate().


def inverse_model(
    robot_x: float,
    robot_y: float,
    robot_heading_rad: float,
    tx_deg: float,
    ty_deg: float,
) -> Tuple[float, float]:
    """
    Re-derive gamepiece field position from Limelight angles.
    Mirrors GamepieceAssistDrive.estimateGamepieceRobotRelative() exactly.
    """
    total_pitch_deg = CAM_PITCH_DEG + ty_deg
    total_pitch_rad = math.radians(total_pitch_deg)

    # Guard against nearly-horizontal ray (avoids division by ~zero)
    tan_pitch = math.tan(total_pitch_rad)
    if abs(tan_pitch) < 1e-3:
        tan_pitch = math.copysign(1e-3, tan_pitch if tan_pitch != 0.0 else 1.0)

    cam_to_piece_dist = (CAM_HEIGHT_M - PIECE_HEIGHT_M) / tan_pitch
    cam_to_piece_dist = max(0.0, cam_to_piece_dist)

    bearing_rad = math.radians(CAM_YAW_DEG + tx_deg)

    # Robot-relative position of the piece
    forward_rel = CAM_FORWARD_M + cam_to_piece_dist * math.cos(bearing_rad)
    left_rel = CAM_LEFT_M + cam_to_piece_dist * math.sin(bearing_rad)

    # Rotate into field frame and add robot translation
    cos_h = math.cos(robot_heading_rad)
    sin_h = math.sin(robot_heading_rad)
    field_x = robot_x + forward_rel * cos_h - left_rel * sin_h
    field_y = robot_y + forward_rel * sin_h + left_rel * cos_h

    return field_x, field_y


# ─── assist vector ─────────────────────────────────────────────────────────────
# Mirrors GamepieceAssistDrive.calculateAssistFieldVelocity() (zero robot velocity).


def assist_velocity(
    robot_x: float, robot_y: float, piece_x: float, piece_y: float
) -> Tuple[float, float]:
    """Compute pursuit assist vector (assumes robot is stationary for demo)."""
    err_x = piece_x - robot_x
    err_y = piece_y - robot_y
    dist = math.hypot(err_x, err_y)
    if dist < STOP_DIST_M:
        return 0.0, 0.0
    vx = PURSUIT_GAIN * err_x
    vy = PURSUIT_GAIN * err_y
    mag = math.hypot(vx, vy)
    if mag > MAX_ASSIST_MPS:
        vx = vx * MAX_ASSIST_MPS / mag
        vy = vy * MAX_ASSIST_MPS / mag
    return vx, vy


# ─── path generation ──────────────────────────────────────────────────────────


def build_smooth_path(
    waypoints: List[Tuple[float, float]], total_steps: int
) -> List[Tuple[float, float, float]]:
    """
    Linear-interpolate waypoints into `total_steps` poses.
    Heading = direction of travel, computed from consecutive positions.
    Returns list of (x, y, heading_rad).
    """
    pts = np.array(waypoints)
    # Cumulative arc length as parameter
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(segs)])
    arc /= arc[-1]  # normalise to [0, 1]

    t = np.linspace(0.0, 1.0, total_steps, endpoint=False)
    x_path = np.interp(t, arc, pts[:, 0])
    y_path = np.interp(t, arc, pts[:, 1])

    # Heading from central finite difference (wraps at ends)
    dx = np.gradient(x_path)
    dy = np.gradient(y_path)
    headings = np.arctan2(dy, dx)

    return list(zip(x_path.tolist(), y_path.tolist(), headings.tolist()))


# ─── FOV polygon ──────────────────────────────────────────────────────────────


def fov_polygon(robot_x, robot_y, robot_heading_rad, range_m=MAX_RANGE_M):
    """
    Returns a filled polygon (list of (x,y)) representing the camera FOV cone
    projected onto the carpet.
    """
    cam_heading = robot_heading_rad + math.radians(CAM_YAW_DEG)
    half_h = math.radians(CAM_HFOV_DEG / 2.0)

    # Range limited by vertical FOV too: when ty = -VFOV/2, use that distance
    ty_min_rad = math.radians(-CAM_VFOV_DEG / 2.0)
    total_pitch_at_bottom = math.radians(CAM_PITCH_DEG) + ty_min_rad
    if abs(math.tan(total_pitch_at_bottom)) > 1e-6:
        fov_range = (CAM_HEIGHT_M - PIECE_HEIGHT_M) / math.tan(total_pitch_at_bottom)
    else:
        fov_range = range_m
    fov_range = min(fov_range, range_m)
    fov_range = max(0.5, fov_range)

    # Camera origin
    cx = robot_x + CAM_FORWARD_M * math.cos(robot_heading_rad) - CAM_LEFT_M * math.sin(robot_heading_rad)
    cy = robot_y + CAM_FORWARD_M * math.sin(robot_heading_rad) + CAM_LEFT_M * math.cos(robot_heading_rad)

    left_edge = (
        cx + fov_range * math.cos(cam_heading + half_h),
        cy + fov_range * math.sin(cam_heading + half_h),
    )
    right_edge = (
        cx + fov_range * math.cos(cam_heading - half_h),
        cy + fov_range * math.sin(cam_heading - half_h),
    )
    return [(cx, cy), left_edge, right_edge]


# ─── matplotlib visualizer ────────────────────────────────────────────────────

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrowPatch, Polygon


def run_matplotlib_demo(path, noise_sigma, speed_factor, save_path):
    matplotlib.rcParams["toolbar"] = "None"
    plt.style.use("dark_background")

    fig = plt.figure(figsize=(16, 7), facecolor="#0d0d0d")
    fig.suptitle(
        "GamepieceAssistDrive  ·  Camera Math Tester",
        color="white",
        fontsize=13,
        fontweight="bold",
        y=0.98,
    )

    # Left: field view
    ax = fig.add_axes([0.02, 0.05, 0.62, 0.90])
    ax.set_facecolor("#1a1a2e")
    ax.set_xlim(0, FIELD_W)
    ax.set_ylim(0, FIELD_H)
    ax.set_aspect("equal")
    ax.tick_params(colors="#555555")
    ax.spines[:].set_color("#333333")
    for spine in ax.spines.values():
        spine.set_linewidth(0.5)
    # Field outline
    field_rect = plt.Rectangle((0, 0), FIELD_W, FIELD_H, fill=False, edgecolor="#444466", lw=1.0)
    ax.add_patch(field_rect)
    # Grid
    for gx in np.arange(0, FIELD_W + 1, 2):
        ax.axvline(gx, color="#222233", lw=0.4, zorder=0)
    for gy in np.arange(0, FIELD_H + 1, 2):
        ax.axhline(gy, color="#222233", lw=0.4, zorder=0)
    ax.set_xlabel("Field X (m)", color="#888888", fontsize=8)
    ax.set_ylabel("Field Y (m)", color="#888888", fontsize=8)

    # Right: data panel
    ax_data = fig.add_axes([0.67, 0.05, 0.31, 0.90])
    ax_data.set_facecolor("#111118")
    ax_data.axis("off")
    ax_data.set_xlim(0, 1)
    ax_data.set_ylim(0, 1)

    # Patch for FOV cone
    fov_poly = Polygon([(0, 0)] * 3, closed=True, facecolor="#1e3a5f", edgecolor="#3a7fbf", alpha=0.5, lw=1.0, zorder=2)
    ax.add_patch(fov_poly)

    # True gamepiece markers
    true_xs = [g[0] for g in GAMEPIECES]
    true_ys = [g[1] for g in GAMEPIECES]
    true_scatter = ax.scatter(true_xs, true_ys, s=120, c="#00e676", marker="o", zorder=5, label="True position", edgecolors="#007a3d", linewidths=1.2)

    # Estimated gamepiece markers (initially empty)
    est_scatter = ax.scatter([], [], s=100, c="#ff6d00", marker="x", zorder=6, label="Camera estimate", linewidths=2.0)

    # Error lines (true ↔ estimated) — list of Line2D artists updated per frame
    error_lines = []

    # Robot body
    robot_body = plt.Circle((0, 0), 0.45, color="#4a90d9", zorder=7, alpha=0.9)
    ax.add_patch(robot_body)

    # Robot heading arrow
    robot_arrow = ax.annotate(
        "",
        xy=(0.5, 0),
        xytext=(0, 0),
        arrowprops=dict(arrowstyle="-|>", color="#ffffff", lw=1.5),
        zorder=8,
    )

    # Assist vector arrow
    assist_arrow = ax.annotate(
        "",
        xy=(0, 0),
        xytext=(0, 0),
        arrowprops=dict(arrowstyle="-|>", color="#ffeb3b", lw=2.0, mutation_scale=20),
        zorder=8,
    )

    # In-FOV highlight rings on true pieces
    fov_rings = [ax.add_patch(plt.Circle(GAMEPIECES[i], 0.20, color="#00e676", fill=False, lw=1.5, zorder=4, alpha=0.0)) for i in range(len(GAMEPIECES))]

    ax.legend(loc="upper right", fontsize=7, framealpha=0.3, labelcolor="white")

    # ── data panel labels ──
    panel_title = ax_data.text(0.5, 0.97, "Live Detection Data", ha="center", va="top", color="white", fontsize=10, fontweight="bold")

    def _label(y, key, col="#aaaaaa"):
        return ax_data.text(0.05, y, key, va="center", color=col, fontsize=8, fontfamily="monospace")

    def _val(y, col="#ffffff"):
        return ax_data.text(0.95, y, "—", va="center", ha="right", color=col, fontsize=9, fontfamily="monospace", fontweight="bold")

    rows = [
        ("Has target",    0.90, "#00e676"),
        ("In FOV count",  0.84, "#80cbc4"),
        ("tx (deg)",      0.76, "#64b5f6"),
        ("ty (deg)",      0.70, "#64b5f6"),
        ("Range (m)",     0.63, "#ce93d8"),
        ("Error (m)",     0.56, "#ff6d00"),
        ("Assist |v|",    0.49, "#ffeb3b"),
        ("Robot X (m)",   0.40, "#aaaaaa"),
        ("Robot Y (m)",   0.34, "#aaaaaa"),
        ("Heading (°)",   0.28, "#aaaaaa"),
    ]
    label_artists = []
    val_artists = []
    for (key, y, col) in rows:
        label_artists.append(_label(y, key, col))
        val_artists.append(_val(y))

    # Separator lines
    for y in [0.93, 0.80, 0.45]:
        ax_data.axhline(y, color="#333344", lw=0.8)

    noise_note = ax_data.text(
        0.5, 0.22,
        f"Camera noise σ = {noise_sigma:.1f}°" if noise_sigma > 0 else "No camera noise",
        ha="center", va="center", color="#888888", fontsize=7, style="italic",
    )

    ax_data.text(0.5, 0.14, "Green  = True position", ha="center", color="#00e676", fontsize=7)
    ax_data.text(0.5, 0.09, "Orange X = Camera estimate", ha="center", color="#ff6d00", fontsize=7)
    ax_data.text(0.5, 0.04, "Yellow -> = Assist vector", ha="center", color="#ffeb3b", fontsize=7)

    # ── error history for rolling stats ──
    error_history = []
    rng = np.random.default_rng(42)
    frame_step = max(1, int(speed_factor))

    def update(frame_idx):
        nonlocal error_lines
        step = (frame_idx * frame_step) % len(path)
        rx, ry, rh = path[step]

        # ── compute detections for all pieces ──
        detections = []  # (tx, ty, true_dist, piece_idx)
        for i, (gx, gy) in enumerate(GAMEPIECES):
            tx, ty, in_fov = forward_model(rx, ry, rh, gx, gy)
            if in_fov:
                # Add camera measurement noise
                if noise_sigma > 0:
                    tx += rng.normal(0, noise_sigma)
                    ty += rng.normal(0, noise_sigma)
                true_dist = math.hypot(gx - rx, gy - ry)
                detections.append((tx, ty, true_dist, i, gx, gy))

        # ── pick "best" detection (closest = largest apparent area) ──
        best = None
        est_poses = []
        if detections:
            best = min(detections, key=lambda d: d[2])  # closest
            for (tx, ty, true_dist, pidx, gx, gy) in detections:
                ex, ey = inverse_model(rx, ry, rh, tx, ty)
                est_poses.append((ex, ey))

        # ── update FOV polygon ──
        fov_pts = fov_polygon(rx, ry, rh)
        fov_poly.set_xy(fov_pts)

        # ── update in-FOV highlight rings ──
        in_fov_set = {d[3] for d in detections}
        for i, ring in enumerate(fov_rings):
            ring.set_alpha(0.7 if i in in_fov_set else 0.0)

        # ── update robot position / heading ──
        robot_body.center = (rx, ry)
        head_len = 0.6
        hx = rx + head_len * math.cos(rh)
        hy = ry + head_len * math.sin(rh)
        robot_arrow.set_position((rx, ry))
        robot_arrow.xy = (hx, hy)
        robot_arrow.xytext = (rx, ry)

        # ── update estimated markers ──
        if est_poses:
            est_scatter.set_offsets(np.array(est_poses))
        else:
            est_scatter.set_offsets(np.empty((0, 2)))

        # ── update error lines ──
        for ln in error_lines:
            ln.remove()
        error_lines = []
        for (tx, ty, true_dist, pidx, gx, gy), (ex, ey) in zip(detections, est_poses):
            ln, = ax.plot([gx, ex], [gy, ey], color="#ff6d00", lw=0.8, alpha=0.5, zorder=3)
            error_lines.append(ln)

        # ── update assist arrow (based on best detection) ──
        if best is not None:
            tx_b, ty_b, _, _, _, _ = best
            ex, ey = inverse_model(rx, ry, rh, tx_b, ty_b)
            avx, avy = assist_velocity(rx, ry, ex, ey)
            assist_mag = math.hypot(avx, avy)
            # Scale the arrow for visibility (1 m/s = 0.5 m on field)
            scale = 0.5 / max(1.0, assist_mag)
            assist_arrow.set_position((rx, ry))
            assist_arrow.xy = (rx + avx * scale, ry + avy * scale)
            assist_arrow.xytext = (rx, ry)
        else:
            # Hide the arrow
            assist_arrow.xy = (rx, ry)
            assist_arrow.xytext = (rx, ry)

        # ── update data panel ──
        has_target = best is not None
        in_fov_count = len(detections)

        if has_target:
            tx_b, ty_b, true_dist_b, pidx_b, gx_b, gy_b = best
            ex_b, ey_b = inverse_model(rx, ry, rh, tx_b, ty_b)
            error_m = math.hypot(ex_b - gx_b, ey_b - gy_b)
            error_history.append(error_m)
            avx, avy = assist_velocity(rx, ry, ex_b, ey_b)
            assist_spd = math.hypot(avx, avy)
            vals = [
                "YES" if has_target else "NO",
                str(in_fov_count),
                f"{tx_b:+.2f}",
                f"{ty_b:+.2f}",
                f"{true_dist_b:.3f}",
                f"{error_m:.4f}",
                f"{assist_spd:.2f}",
                f"{rx:.3f}",
                f"{ry:.3f}",
                f"{math.degrees(rh):.1f}",
            ]
        else:
            vals = ["NO", "0", "—", "—", "—", "—", "—", f"{rx:.3f}", f"{ry:.3f}", f"{math.degrees(rh):.1f}"]

        for art, v in zip(val_artists, vals):
            art.set_text(v)
            if v == "YES":
                art.set_color("#00e676")
            elif v == "NO":
                art.set_color("#ef5350")
            else:
                art.set_color("#ffffff")

        return [fov_poly, robot_body, est_scatter, assist_arrow] + error_lines + fov_rings

    total_frames = len(path) // frame_step + 1
    interval_ms = max(16, ANIM_MS // frame_step)
    ani = FuncAnimation(fig, update, frames=total_frames, interval=interval_ms, blit=False, repeat=True)

    if save_path:
        print(f"Saving animation to {save_path} ...  (this may take a minute)")
        ani.save(save_path, writer="pillow", fps=20, dpi=90)
        print("Saved.")
    else:
        plt.show()

    return ani  # keep reference alive


# ─── NetworkTables publisher ───────────────────────────────────────────────────


def run_nt_loop(path, noise_sigma, speed_factor):
    """Publishes simulation state to NT4 so AdvantageScope can display it."""
    if not NT_AVAILABLE:
        print("[NT] ERROR: 'ntcore' package not found.  Install with:  pip install robotpy-ntcore")
        return

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer(listenAddress="0.0.0.0", port3=0, port4=5810)
    print("[NT] Server started on NT4 port 5810 — connect AdvantageScope to localhost:5810")

    field = inst.getTable("SmartDashboard/Field")
    vt = inst.getTable("SmartDashboard/VisionTest")

    robot_pub = field.getDoubleArrayTopic("Robot").publish()
    true_pub = field.getDoubleArrayTopic("TrueGamepieces").publish()
    est_pub = field.getDoubleArrayTopic("EstimatedPieces").publish()
    assist_pub = field.getDoubleArrayTopic("AssistArrow").publish()
    fov_pub = field.getDoubleArrayTopic("CameraFOV").publish()

    tx_pub = vt.getDoubleTopic("TxDeg").publish()
    ty_pub = vt.getDoubleTopic("TyDeg").publish()
    range_pub = vt.getDoubleTopic("RangeMeters").publish()
    error_pub = vt.getDoubleTopic("ErrorMeters").publish()
    has_pub = vt.getBooleanTopic("HasTarget").publish()
    count_pub = vt.getIntegerTopic("InFOV").publish()

    # Publish fixed true piece poses
    true_arr = []
    for gx, gy in GAMEPIECES:
        true_arr += [gx, gy, 0.0]  # x, y, rot_deg (Field2d convention)
    true_pub.set(true_arr)

    rng = np.random.default_rng(0)
    step = 0
    dt = 1.0 / (LOOP_HZ * speed_factor)
    try:
        while True:
            rx, ry, rh = path[step % len(path)]
            step += 1

            detections = []
            for i, (gx, gy) in enumerate(GAMEPIECES):
                tx, ty, in_fov = forward_model(rx, ry, rh, gx, gy)
                if in_fov:
                    if noise_sigma > 0:
                        tx += rng.normal(0, noise_sigma)
                        ty += rng.normal(0, noise_sigma)
                    detections.append((tx, ty, math.hypot(gx - rx, gy - ry), gx, gy))

            best = min(detections, key=lambda d: d[2]) if detections else None

            # Robot pose
            robot_pub.set([rx, ry, math.degrees(rh)])

            # Estimated poses array
            est_arr = []
            for tx, ty, _, gx, gy in detections:
                ex, ey = inverse_model(rx, ry, rh, tx, ty)
                est_arr += [ex, ey, 0.0]
            est_pub.set(est_arr if est_arr else [])

            # Assist arrow (robot pose + target pose as two-element pose array)
            if best is not None:
                tx_b, ty_b, _, gx_b, gy_b = best
                ex, ey = inverse_model(rx, ry, rh, tx_b, ty_b)
                avx, avy = assist_velocity(rx, ry, ex, ey)
                assist_pub.set([rx, ry, math.degrees(math.atan2(avy, avx)), ex, ey, 0.0])
                error_m = math.hypot(ex - gx_b, ey - gy_b)
                tx_pub.set(tx_b)
                ty_pub.set(ty_b)
                range_pub.set(math.hypot(gx_b - rx, gy_b - ry))
                error_pub.set(error_m)
                has_pub.set(True)
            else:
                assist_pub.set([])
                error_pub.set(0.0)
                has_pub.set(False)

            count_pub.set(len(detections))

            # FOV cone (3-point polygon published as 3 "poses", rotation unused)
            pts = fov_polygon(rx, ry, rh)
            fov_arr = []
            for px, py in pts:
                fov_arr += [px, py, 0.0]
            fov_pub.set(fov_arr)

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[NT] Stopping.")


# ─── math self-test ───────────────────────────────────────────────────────────


def run_self_test():
    """
    Verify that forward_model -> inverse_model round-trips with near-zero error
    for a grid of robot poses and gamepiece positions.
    """
    import itertools

    print("Running self-test (forward -> inverse round-trip) ...")
    max_err = 0.0
    tested = 0
    for rx, ry, rh_deg, gx, gy in itertools.product(
        [2.0, 5.0, 8.0, 12.0],
        [1.5, 4.1, 6.7],
        [0, 45, 90, 180, 270],
        [1.0, 4.0, 7.0, 13.0],
        [1.0, 4.1, 7.2],
    ):
        rh = math.radians(rh_deg)
        tx, ty, in_fov = forward_model(rx, ry, rh, gx, gy)
        if not in_fov:
            continue
        ex, ey = inverse_model(rx, ry, rh, tx, ty)
        err = math.hypot(ex - gx, ey - gy)
        max_err = max(max_err, err)
        tested += 1

    print(f"  Tested {tested} in-FOV cases  |  max round-trip error = {max_err*100:.4f} cm")
    if max_err < 1e-4:
        print("  PASS -- forward/inverse math is internally consistent (check)")
    else:
        print("  WARN -- error exceeds 0.1 mm; check trig sign conventions")


# ─── entry point ──────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nt", action="store_true", help="Also publish state to NetworkTables (NT4, port 5810) for AdvantageScope")
    parser.add_argument("--noise", type=float, default=0.0, metavar="SIGMA", help="Gaussian noise σ added to tx/ty (degrees).  Try 0.5–2.0 for realistic camera noise.")
    parser.add_argument("--speed", type=float, default=1.0, metavar="X", help="Animation playback speed multiplier (default 1.0)")
    parser.add_argument("--save", type=str, default="", metavar="FILE", help="Save animation to FILE (e.g. demo.gif) instead of showing window")
    parser.add_argument("--test", action="store_true", help="Run forward/inverse round-trip self-test and exit")
    args = parser.parse_args()

    if args.test:
        run_self_test()
        sys.exit(0)

    # Print camera constants in use
    print("=" * 60)
    print("  GamepieceAssistDrive  Camera Math Tester")
    print("=" * 60)
    print(f"  Camera pitch:   {CAM_PITCH_DEG}°   (positive = looking down)")
    print(f"  Camera yaw:     {CAM_YAW_DEG}°   (positive = pointing left)")
    print(f"  Camera height:  {CAM_HEIGHT_M} m")
    print(f"  Camera forward: {CAM_FORWARD_M} m  left: {CAM_LEFT_M} m")
    print(f"  Piece height:   {PIECE_HEIGHT_M} m")
    print(f"  Camera HFOV:    {CAM_HFOV_DEG}°   VFOV: {CAM_VFOV_DEG}°")
    print(f"  Max range:      {MAX_RANGE_M} m")
    if args.noise > 0:
        print(f"  Noise sigma:    {args.noise} deg added to tx / ty")
    print("=" * 60)

    # Build dense path
    path = build_smooth_path(RAW_WAYPOINTS, total_steps=2000)
    print(f"  Path:  {len(RAW_WAYPOINTS)} waypoints -> {len(path)} path steps")
    print(f"  Pieces: {len(GAMEPIECES)}")
    print()

    if args.nt:
        if not NT_AVAILABLE:
            print("  [!] robotpy-ntcore not installed -- NT output disabled.")
            print("      Run:  pip install robotpy-ntcore")
            print()
        else:
            import threading
            t = threading.Thread(target=run_nt_loop, args=(path, args.noise, args.speed), daemon=True)
            t.start()
            print("  [NT] Background NT publisher started on port 5810.")

    run_matplotlib_demo(path, args.noise, args.speed, args.save if args.save else None)


if __name__ == "__main__":
    main()
