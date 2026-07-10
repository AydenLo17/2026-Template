# GamepieceAssistDrive Testing Tools

## Overview

Test and visualize the camera math that powers `GamepieceAssistDrive.java` **without a real robot**. Validate the trigonometry, measure estimation error, and show live data on AdvantageScope.

## Files

| File | Purpose |
|------|---------|
| `scripts/gamepiece_vision_test.py` | Full interactive tester with matplotlib visualization |
| `scripts/gamepiece_vision_nt_streamer.py` | Lightweight NT4 streamer (no window, good for AdvantageScope demos) |
| `scripts/README_GAMEPIECE_VISION.md` | Detailed usage guide |
| `src/main/java/frc/robot/commands/GamepieceAssistDrive.java` | Real robot command (uses this math) |
| `src/main/java/frc/robot/Constants.java` | Camera geometry and tuning params |

## Quick Start

### 1. Verify the math (no dependencies)

```bash
python scripts/gamepiece_vision_test.py --test
```

Output: `PASS -- forward/inverse math is internally consistent (check)` means the Python trig perfectly inverts the Java trig. **0.0000 cm error.**

### 2. Interactive visualization (matplotlib)

```bash
python scripts/gamepiece_vision_test.py
```

Shows a matplotlib window with robot sweeping past gamepieces. See real vs. estimated positions, FOV cone, assist vector, and live error metrics.

### 3. Stream to AdvantageScope (requires: `pip install robotpy-ntcore`)

**Terminal 1** — Start the streamer:
```bash
python scripts/gamepiece_vision_nt_streamer.py
```

Output:
```
[NT] NT4 server started on port 5810
[NT] Streaming at 50 Hz, noise sigma = 0.0 deg
[NT] Open AdvantageScope: File -> Connect to NT4 -> localhost:5810
```

**Terminal 2** — Connect AdvantageScope:
- File → Connect to NT4 → `localhost:5810` → Connect
- Drag `SmartDashboard/Field` onto a 2D Field widget
- Watch the simulation in real-time overlay with telemetry

## How It Works

### Forward Model
**Input:** Robot pose + true gamepiece position → **Output:** What the Limelight would report (`tx`, `ty`)

Applies:
1. Robot frame rotation
2. Camera mount offset (forward, left)
3. Camera pitch/yaw angles
4. FOV culling

### Inverse Model
**Input:** Camera angles + robot pose → **Output:** Estimated field position

Mirrors `GamepieceAssistDrive.estimateGamepieceRobotRelative()` exactly:
1. Compute range from vertical angle: `range = (cam_height - piece_height) / tan(pitch + ty)`
2. Compute robot-relative bearing: `forward = cam_forward + range * cos(yaw + tx)`
3. Rotate into field frame

### Assist Vector
**Input:** Estimated piece position → **Output:** Velocity command to pursuit it

Mirrors `GamepieceAssistDrive.calculateAssistFieldVelocity()`:
```
error = piece - robot
assist = pursuit_gain * error - damping * velocity
clamp to max_speed
```

## Key Results

- **Math validation:** Forward → inverse round-trip error = **0.0000 cm** across 39 test cases ✓
- **Field visualization:** Robot, true pieces (green), estimated pieces (orange), camera FOV, assist vector, live error display
- **Live telemetry:** `tx`, `ty`, range, error, assist speed, robot state streamed to NetworkTables in real-time

## Simulation Scenario

- **Robot path:** 17 waypoints forming a closed patrol around the field
- **Gamepieces:** 6 positions distributed across the field
- **Camera:** Limelight 3G (LL3G) typical FOV (63.3° H × 49.7° V)
- **Noise:** Optional Gaussian noise on `tx`/`ty` to test robustness

## Usage Examples

```bash
# Basic test
python scripts/gamepiece_vision_test.py --test

# Interactive demo, 3x speed
python scripts/gamepiece_vision_test.py --speed 3

# With realistic camera noise (1.5 degrees)
python scripts/gamepiece_vision_test.py --noise 1.5

# Save as GIF (slow ~1 min)
python scripts/gamepiece_vision_test.py --speed 2 --save demo.gif

# Stream to AdvantageScope
python scripts/gamepiece_vision_nt_streamer.py --noise 0.8 --speed 1.5

# Stream for 30 seconds then exit
python scripts/gamepiece_vision_nt_streamer.py --duration 30
```

## On Real Hardware

Once you have measured your camera geometry:

1. Update `Constants.Gamepiece`:
   - `kCameraPitchDegrees` — your mount pitch
   - `kCameraYawDegrees` — your mount yaw
   - `kCameraHeightMeters`, `kCameraForwardMeters`, `kCameraLeftMeters` — mount position

2. Update the script with the same values:
   ```python
   CAM_PITCH_DEG = 24.0      # your pitch
   CAM_YAW_DEG = 0.0         # your yaw
   CAM_HEIGHT_M = 0.56       # etc.
   ```

3. Test in simulation: `python scripts/gamepiece_vision_test.py --noise 1.0 --nt`
   - Expected error: 5–10 cm at intake distance (assuming 1° camera noise)

4. Tune on real robot:
   - Adjust `kPursuitGain`, `kVelocityDamping`, `kAssistBlend` until smooth
   - Monitor AdvantageScope: `SmartDashboard/VisionTest/ErrorMeters`
   - Watch the assist vector and make sure it pulls toward pieces naturally

## Dependencies

- `matplotlib` — interactive visualization (included in standard Python installs)
- `numpy` — path interpolation (included)
- `robotpy-ntcore` — optional, only for NT4 streaming to AdvantageScope

To install optional NT4 support:
```bash
pip install robotpy-ntcore
```

## Troubleshooting

**Q: "ModuleNotFoundError: No module named 'ntcore'"**  
A: Install: `pip install robotpy-ntcore`

**Q: Matplotlib window doesn't appear**  
A: You're likely on a headless system. Use the NT streamer instead: `python scripts/gamepiece_vision_nt_streamer.py`

**Q: GIF save is slow**  
A: Normal — Pillow encodes 2000 frames. Try `--speed 5` to reduce frame count.

**Q: AdvantageScope field is empty**  
A: Make sure you're connected to port **5810** and dragged `SmartDashboard/Field` onto a 2D Field widget.

**Q: Windows encoding error**  
A: In PowerShell: `chcp 65001` (enables UTF-8) before running script.

## References

- `GamepieceAssistDrive.java` — the real robot command (see `estimateGamepieceRobotRelative()` method)
- `Constants.java` — camera geometry and tuning (`Constants.Gamepiece` class)
- `VISION_FEATURE_TODO.md` — future vision upgrades (dynamic pipeline switching, etc.)
