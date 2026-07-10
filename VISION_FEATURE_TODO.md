# Vision + Estimator Feature Todo

## Disturbance Detection (wall hit, wheel slip, estimator stability)

- [ ] Add detection constants to `Constants.Estimator` (slip thresholds, impact accel/yaw thresholds, hold windows).
- [ ] Add a disturbance monitor implementation that reads commanded vs measured velocity and yaw dynamics.
- [ ] Publish disturbance telemetry (`SlipActive`, `ImpactActive`, `VisionUnstable`) for tuning.
- [ ] Apply temporary trust multipliers in estimator fusion when disturbance flags are active.
- [ ] Apply short-lived vision trust adjustments when impact/unstable vision is detected.
- [ ] Validate in logs and tune thresholds on real robot.

## Dynamic Limelight Crop Window (distance-based)

- [x] Add tuning constants for crop sizes, distance breakpoints, and update deadband.
- [x] Simplify tuning surface to core knobs (near/far distance + near/far half-size + deadband).
- [x] Implement crop window selection from tag distance (larger window close, tighter window far).
- [x] Center crop around detected target (`txnc`/`tync`) with safe clamping to `[-1, 1]`.
- [x] Add fallback behavior when no valid target is detected (full-frame crop).
- [x] Publish crop telemetry (`CropXMin/XMax/YMin/YMax`, `CropScale`, `CropMode`) to `NT:/Vision/<camera>/*`.
- [x] Add optional enable flag so feature can be toggled during testing.
- [ ] Validate latency/FPS impact and pose quality on real robot.

## Dynamic Limelight Tag Pipeline Switching (future upgrade)

- [ ] Add near/far tag pipelines tuned for close control vs high-speed/far detection.
- [ ] Add hysteresis and min-hold timing to prevent pipeline thrash.
- [ ] Decide switching inputs (speed, recent tag distance, and optional match phase).
- [ ] Publish pipeline telemetry (`PipelineIndex`, `PipelineMode`) to `NT:/Vision/<camera>/*`.
- [ ] Validate whether switching improves detection consistency over crop-only behavior.
