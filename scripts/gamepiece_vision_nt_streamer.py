#!/usr/bin/env python3
"""
scripts/gamepiece_vision_nt_streamer.py
========================================
Simpler version of gamepiece_vision_test.py that ONLY streams simulation state to NetworkTables.
No matplotlib window — perfect for live AdvantageScope visualization on headless systems.

USAGE
  python scripts/gamepiece_vision_nt_streamer.py                 # default: 50 Hz loop
  python scripts/gamepiece_vision_nt_streamer.py --noise 1.5     # add tx/ty noise
  python scripts/gamepiece_vision_nt_streamer.py --speed 2       # run 2x faster
  python scripts/gamepiece_vision_nt_streamer.py --duration 60   # run for 60 seconds then stop

ADVANTAGESCOPE SETUP
  1. Run this script in one terminal:     python scripts/gamepiece_vision_nt_streamer.py
  2. In another terminal, run AdvantageScope and connect to NT4 -> localhost:5810
  3. Drag SmartDashboard/Field onto a 2D Field widget
  4. Watch the robot animate past gamepieces in real-time
"""

import argparse
import math
import signal
import sys
import time
from typing import List, Tuple

import numpy as np

try:
    import ntcore
    NT_AVAILABLE = True
except ImportError:
    NT_AVAILABLE = False
    print("[!] robotpy-ntcore not found. Install with: pip install robotpy-ntcore")
    sys.exit(1)

# Camera constants (mirror Constants.Gamepiece)
CAM_PITCH_DEG = 24.0
CAM_YAW_DEG = 0.0
CAM_HEIGHT_M = 0.56
CAM_FORWARD_M = 0.30
CAM_LEFT_M = 0.00
PIECE_HEIGHT_M = 0.02
CAM_HFOV_DEG = 63.3
CAM_VFOV_DEG = 49.7
MAX_RANGE_M = 4.5

# Assist constants
STOP_DIST_M = 0.22
PURSUIT_GAIN = 1.8
MAX_ASSIST_MPS = 1.8

# Field
FIELD_W = 16.54
FIELD_H = 8.21

# Demo scenario
GAMEPIECES = [
    (2.50, 1.80),
    (2.50, 4.10),
    (2.50, 6.30),
    (5.00, 2.70),
    (5.00, 5.50),
    (8.27, 4.10),
]

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


def forward_model(robot_x, robot_y, robot_heading_rad, piece_x, piece_y):
    """Synthetic Limelight angles."""
    cam_x = robot_x + CAM_FORWARD_M * math.cos(robot_heading_rad) - CAM_LEFT_M * math.sin(robot_heading_rad)
    cam_y = robot_y + CAM_FORWARD_M * math.sin(robot_heading_rad) + CAM_LEFT_M * math.cos(robot_heading_rad)

    dx = piece_x - cam_x
    dy = piece_y - cam_y

    cam_heading = robot_heading_rad + math.radians(CAM_YAW_DEG)
    fwd = dx * math.cos(cam_heading) + dy * math.sin(cam_heading)
    lat = -dx * math.sin(cam_heading) + dy * math.cos(cam_heading)

    if fwd <= 0.05:
        return 0.0, 0.0, False

    horiz_dist = math.sqrt(fwd * fwd + lat * lat)
    tx_rad = math.atan2(lat, fwd)
    tx_deg = math.degrees(tx_rad) - CAM_YAW_DEG

    height_diff = CAM_HEIGHT_M - PIECE_HEIGHT_M
    total_pitch_deg = math.degrees(math.atan2(height_diff, horiz_dist))
    ty_deg = total_pitch_deg - CAM_PITCH_DEG

    half_hfov = CAM_HFOV_DEG / 2.0
    half_vfov = CAM_VFOV_DEG / 2.0
    in_fov = (
        abs(tx_deg) <= half_hfov
        and abs(ty_deg) <= half_vfov
        and horiz_dist <= MAX_RANGE_M
    )

    return tx_deg, ty_deg, in_fov


def inverse_model(robot_x, robot_y, robot_heading_rad, tx_deg, ty_deg):
    """Recover field position from camera angles."""
    total_pitch_deg = CAM_PITCH_DEG + ty_deg
    total_pitch_rad = math.radians(total_pitch_deg)

    tan_pitch = math.tan(total_pitch_rad)
    if abs(tan_pitch) < 1e-3:
        tan_pitch = math.copysign(1e-3, tan_pitch if tan_pitch != 0.0 else 1.0)

    cam_to_piece_dist = (CAM_HEIGHT_M - PIECE_HEIGHT_M) / tan_pitch
    cam_to_piece_dist = max(0.0, cam_to_piece_dist)

    bearing_rad = math.radians(CAM_YAW_DEG + tx_deg)

    forward_rel = CAM_FORWARD_M + cam_to_piece_dist * math.cos(bearing_rad)
    left_rel = CAM_LEFT_M + cam_to_piece_dist * math.sin(bearing_rad)

    cos_h = math.cos(robot_heading_rad)
    sin_h = math.sin(robot_heading_rad)
    field_x = robot_x + forward_rel * cos_h - left_rel * sin_h
    field_y = robot_y + forward_rel * sin_h + left_rel * cos_h

    return field_x, field_y


def assist_velocity(robot_x, robot_y, piece_x, piece_y):
    """Compute pursuit assist vector."""
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


def build_smooth_path(waypoints: List[Tuple[float, float]], total_steps: int) -> List[Tuple[float, float, float]]:
    """Smooth waypoint path."""
    pts = np.array(waypoints)
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(segs)])
    arc /= arc[-1]

    t = np.linspace(0.0, 1.0, total_steps, endpoint=False)
    x_path = np.interp(t, arc, pts[:, 0])
    y_path = np.interp(t, arc, pts[:, 1])

    dx = np.gradient(x_path)
    dy = np.gradient(y_path)
    headings = np.arctan2(dy, dx)

    return list(zip(x_path.tolist(), y_path.tolist(), headings.tolist()))


def fov_polygon(robot_x, robot_y, robot_heading_rad, range_m=MAX_RANGE_M):
    """Camera FOV cone as 3-point polygon."""
    cam_heading = robot_heading_rad + math.radians(CAM_YAW_DEG)
    half_h = math.radians(CAM_HFOV_DEG / 2.0)

    ty_min_rad = math.radians(-CAM_VFOV_DEG / 2.0)
    total_pitch_at_bottom = math.radians(CAM_PITCH_DEG) + ty_min_rad
    if abs(math.tan(total_pitch_at_bottom)) > 1e-6:
        fov_range = (CAM_HEIGHT_M - PIECE_HEIGHT_M) / math.tan(total_pitch_at_bottom)
    else:
        fov_range = range_m
    fov_range = min(fov_range, range_m)
    fov_range = max(0.5, fov_range)

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


def run_nt_streamer(path, noise_sigma, speed_factor, duration_seconds=None):
    """Stream simulation to NT4."""
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer(listenAddress="0.0.0.0", port3=0, port4=5810)
    print(f"[NT] NT4 server started on port 5810")

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

    # True piece poses
    true_arr = []
    for gx, gy in GAMEPIECES:
        true_arr += [gx, gy, 0.0]
    true_pub.set(true_arr)

    rng = np.random.default_rng(0)
    step = 0
    loop_hz = 50
    dt = 1.0 / (loop_hz * speed_factor)
    start_time = time.time()

    print(f"[NT] Streaming at {loop_hz * speed_factor:.0f} Hz, noise sigma = {noise_sigma}°")
    print(f"[NT] Open AdvantageScope: File -> Connect to NT4 -> localhost:5810")
    print()

    try:
        while True:
            if duration_seconds is not None and time.time() - start_time > duration_seconds:
                print(f"[NT] Duration limit ({duration_seconds}s) reached, stopping.")
                break

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

            # Estimated poses
            est_arr = []
            for tx, ty, _, gx, gy in detections:
                ex, ey = inverse_model(rx, ry, rh, tx, ty)
                est_arr += [ex, ey, 0.0]
            est_pub.set(est_arr if est_arr else [])

            # Assist arrow + detection data
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

            # FOV cone
            pts = fov_polygon(rx, ry, rh)
            fov_arr = []
            for px, py in pts:
                fov_arr += [px, py, 0.0]
            fov_pub.set(fov_arr)

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[NT] Interrupted, shutting down.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--noise", type=float, default=0.0, metavar="SIGMA", help="Gaussian noise sigma (degrees)")
    parser.add_argument("--speed", type=float, default=1.0, metavar="X", help="Playback speed multiplier")
    parser.add_argument("--duration", type=float, default=None, metavar="SEC", help="Run for N seconds then exit")
    args = parser.parse_args()

    print("=" * 60)
    print("  GamepieceAssistDrive  NT4 Vision Streamer")
    print("=" * 60)
    print(f"  Camera pitch:   {CAM_PITCH_DEG} deg")
    print(f"  Camera height:  {CAM_HEIGHT_M} m")
    print(f"  Pieces:         {len(GAMEPIECES)}")
    print(f"  Speed:          {args.speed}x  Noise: {args.noise} deg")
    if args.duration:
        print(f"  Duration:       {args.duration}s")
    print("=" * 60)
    print()

    path = build_smooth_path(RAW_WAYPOINTS, total_steps=2000)
    run_nt_streamer(path, args.noise, args.speed, args.duration)


if __name__ == "__main__":
    main()
