# Gamepiece Vision Math Tester

Test and visualize the camera math from `GamepieceAssistDrive` **without a real robot**.

## What This Does

The `gamepiece_vision_test.py` script validates that the trigonometry is correct:

1. **Forward model** — Given a robot pose + true gamepiece position, compute what the Limelight would report (`tx`, `ty`)
2. **Inverse model** — Given those angles, recover the gamepiece's field position  
   *(identical trig to `GamepieceAssistDrive.estimateGamepieceRobotRelative()`)*
3. **Animation** — Show the robot sweeping past 6 gamepieces, overlaying true vs. estimated positions so you can see the estimation error in real-time

## Requirements

```bash
pip install matplotlib numpy
pip install robotpy-ntcore      # optional, only needed for --nt live streaming
```

## Quick Start

### 1. Verify the math is correct

```bash
python scripts/gamepiece_vision_test.py --test
```

Output should show **0.0000 cm max error** — the Python trig perfectly inverts the Java trig.

### 2. Watch the animation (interactive)

```bash
python scripts/gamepiece_vision_test.py
```

A matplotlib window opens showing:
- **Green dots** = true gamepiece positions (field frame)
- **Orange X** = estimated position from camera math
- **Blue circle** = robot with white heading arrow
- **Blue-tinted cone** = camera field of view
- **Yellow arrow** = pursuit assist vector

The right panel shows live data:
- `tx`, `ty` — camera angles to best detection
- `Range`, `Error` — distance and estimation error
- `Assist |v|` — magnitude of assist velocity

Press **Close** to exit.

### 3. Save as GIF (slow ~1–2 min)

```bash
python scripts/gamepiece_vision_test.py --save demo.gif
```

Creates `demo.gif` — shareable with your team to show the math in action.

### 4. Speed up playback

```bash
python scripts/gamepiece_vision_test.py --speed 3 --save fast_demo.gif
```

Runs 3× faster.

### 5. Add camera noise to test robustness

```bash
python scripts/gamepiece_vision_test.py --noise 1.5
```

Adds Gaussian noise σ=1.5° to `tx`/`ty` to simulate real camera jitter. Watch how estimation error changes.

---

## Live Streaming to AdvantageScope

Stream the simulation state to NetworkTables so you can overlay the camera FOV cone, estimated positions, and assist vector on AdvantageScope's field widget.

### Setup

1. **Install dependency** (one-time):
   ```bash
   pip install robotpy-ntcore
   ```

2. **Start the streamer**:
   ```bash
   python scripts/gamepiece_vision_test.py --nt
   ```

   Output:
   ```
   ============================================================
     GamepieceAssistDrive  Camera Math Tester
   ============================================================
   ...
   [NT] Background NT publisher started on port 5810.
   ```

3. **Connect AdvantageScope**:
   - Open AdvantageScope
   - **File** → **Connect to NT4** → Type `localhost:5810` → **Connect**

4. **Drag widgets**:
   - Drag `SmartDashboard/Field` onto a **2D Field** widget
   - (Optional) Drag scalar keys under `SmartDashboard/VisionTest/` onto graphs

### Published Keys

**Field frame poses** (3-element arrays: `[x_m, y_m, rot_deg]`):
- `SmartDashboard/Field/Robot` — current robot pose
- `SmartDashboard/Field/TrueGamepieces` — actual piece positions (N×3 array)
- `SmartDashboard/Field/EstimatedPieces` — camera-estimated positions
- `SmartDashboard/Field/AssistArrow` — assist vector (from robot to target)
- `SmartDashboard/Field/CameraFOV` — triangular FOV cone (3×3 array)

**Scalar/boolean**:
- `SmartDashboard/VisionTest/TxDeg` — horizontal angle to best target (degrees)
- `SmartDashboard/VisionTest/TyDeg` — vertical angle to best target (degrees)
- `SmartDashboard/VisionTest/RangeMeters` — distance to best target
- `SmartDashboard/VisionTest/ErrorMeters` — estimation error magnitude
- `SmartDashboard/VisionTest/HasTarget` — true if camera locked on something
- `SmartDashboard/VisionTest/InFOV` — count of pieces in camera FOV

---

## How the Math Works

### Forward Model (robot pose → camera angles)

```
Input:  robot pose [x, y, heading_rad] + piece position [x, y]

1. Compute piece position in camera frame:
   - Rotate piece by robot heading into body frame
   - Account for camera offset (forward, left)
   - Apply camera yaw/pitch mount angles

2. Compute tx/ty:
   - tx = atan2(lateral_component, forward_component)
   - ty = total_pitch_angle - camera_pitch
```

### Inverse Model (camera angles → field position)

Mirrors `GamepieceAssistDrive.estimateGamepieceRobotRelative()` exactly:

```
Input:  camera angles [tx, ty] + robot pose [x, y, heading_rad]

1. Compute range to piece:
   range = (camera_height - piece_height) / tan(camera_pitch + ty)

2. Compute robot-relative position:
   forward = camera_forward + range * cos(camera_yaw + tx)
   left    = camera_left + range * sin(camera_yaw + tx)

3. Rotate into field frame + add robot translation:
   field_x = robot_x + forward * cos(heading) - left * sin(heading)
   field_y = robot_y + forward * sin(heading) + left * cos(heading)
```

### Assist Vector (estimated position → drive command)

Mirrors `GamepieceAssistDrive.calculateAssistFieldVelocity()`:

```
Input:  robot pose [x, y] + estimated piece position [x, y]

error_vector = piece - robot

if distance < stop_distance:
   assist = [0, 0]
else:
   assist = pursuit_gain * error - damping * robot_velocity
   clamp magnitude to max_assist_speed
```

---

## Validating on Real Hardware

Once you have a real robot with a Limelight:

1. **Measure camera geometry** (all marked `// TODO: measure` in `Constants.Gamepiece`):
   - `kCameraPitchDegrees` — mount pitch, positive = looking down
   - `kCameraYawDegrees` — side tilt, positive = looking left
   - `kCameraHeightMeters` — lens height above carpet
   - `kCameraForwardMeters` — forward distance from robot center
   - `kCameraLeftMeters` — left distance from robot center

2. **Update the script**:
   ```python
   # At top of gamepiece_vision_test.py:
   CAM_PITCH_DEG = 24.0  # your actual pitch
   CAM_YAW_DEG = 0.0     # your actual yaw
   CAM_HEIGHT_M = 0.56
   # etc.
   ```

3. **Test in simulation** to see if error < 5–10 cm at typical intake distances.

4. **Tune on real robot**:
   - Run `python scripts/gamepiece_vision_test.py --noise 1.0 --nt`
   - Drive robot near gamepieces while monitoring AdvantageScope
   - Adjust gains (`kPursuitGain`, `kVelocityDamping`, etc.) until assist feels natural

---

## Troubleshooting

**Error: "NameError: name 'ANIM_MS' not defined"**  
→ Update your script. The constants were added in recent versions.

**GIF save is very slow**  
→ Normal — matplotlib's Pillow GIF encoder is slow for 2000 frames. Either wait or use `--speed 5` to reduce frame count.

**AdvantageScope shows empty field**  
→ Make sure you're connected to port **5810**, and drag `SmartDashboard/Field` onto a 2D Field widget (not the default layout).

**Python encoding error on Windows**  
→ Run `chcp 65001` in PowerShell before running the script (enables UTF-8 console).

---

## Files

- `scripts/gamepiece_vision_test.py` — Main tester (forward/inverse model + animation)
- `src/main/java/frc/robot/commands/GamepieceAssistDrive.java` — Real robot implementation
- `src/main/java/frc/robot/Constants.java` — Camera geometry + tuning params
