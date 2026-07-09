"""Isaac Sim <-> WPILib headless co-simulation bridge (NetworkTables client).

This standalone script runs *inside Isaac Sim's* Python environment. It joins the NetworkTables
server that the headless WPILib simulation stands up, applies the robot's commanded swerve inputs to
the (Isaac / PhysX) physics robot, and feeds a ground-truth gyro heading back so the robot's
``VisionAutoAlign`` command scores its alignment against real physics.

Contract (all keys under the ``SmartDashboard`` table, matching ``VisionAutoAlign.java``):

======================================  =========  ================================================
Key                                     Direction  Meaning
======================================  =========  ================================================
``Swerve/TargetSpeed``                  IN         Commanded yaw rate (rad/s) from the robot.
``Swerve/TargetAngle``                  IN         Goal heading (degrees) the robot is aligning to.
``SimSensors/GyroHeading``              OUT        Ground-truth heading (degrees) from physics.
``Vision/MetricAlignmentTime``          IN         Alignment duration (s) the robot publishes on
                                                   convergence; the bridge echoes it as [AI_METRIC].
======================================  =========  ================================================

Run it via ``run_ai_tuning_cycle.py`` (which launches the WPILib sim first), or manually with the
Isaac Sim python wrapper, e.g. ``%ISAAC_SIM_PYTHON% isaac_sim_bridge.py``.

NOTE: The PhysX joint-drive wiring is intentionally left as a clearly marked placeholder. Until a
USD robot model is loaded, the bridge integrates the commanded yaw rate to produce a plausible
ground-truth heading so the whole loop is exercisable end-to-end. Replace the placeholder block with
reads/writes against your loaded chassis prim once the USD scene exists.
"""

from __future__ import annotations

import argparse
import math
import time


def _boot_isaac(headless: bool):
    """Boot Isaac Sim and return (simulation_app, sim_context) or (None, None) if unavailable."""
    try:
        from isaacsim import SimulationApp  # type: ignore
    except ImportError:
        print("[ISAAC_BRIDGE] Isaac Sim not importable in this interpreter.")
        print("[ISAAC_BRIDGE] Run this file with the Isaac Sim python wrapper (ISAAC_SIM_PYTHON).")
        return None, None

    simulation_app = SimulationApp({"headless": headless})
    # Import core modules only AFTER the app is created (Omniverse requirement).
    from omni.isaac.core import SimulationContext  # type: ignore

    sim_context = SimulationContext(stage_units_in_meters=1.0)
    sim_context.set_physics_dt(0.02)  # match the WPILib 20 ms frame cycle
    return simulation_app, sim_context


def main() -> int:
    parser = argparse.ArgumentParser(description="Isaac Sim <-> WPILib NetworkTables bridge")
    parser.add_argument("--server", default="127.0.0.1", help="NetworkTables server (WPILib sim)")
    parser.add_argument("--max-steps", type=int, default=750, help="Safety step cap (15 s @ 50 Hz)")
    parser.add_argument("--gui", action="store_true", help="Show the Isaac window (default headless)")
    args = parser.parse_args()

    try:
        import ntcore
    except ImportError:
        print("[ISAAC_BRIDGE] The 'ntcore' package is required (pip install robotpy-ntcore).")
        return 1

    print("[ISAAC_BRIDGE] Initializing headless integration environment...")
    simulation_app, sim_context = _boot_isaac(headless=not args.gui)
    physx_available = simulation_app is not None

    # --- NetworkTables 4 client -----------------------------------------------------------------
    nt = ntcore.NetworkTableInstance.getDefault()
    nt.startClient4("IsaacSim_Agent_Bridge")
    nt.setServer(args.server, ntcore.NetworkTableInstance.kDefaultPort4)
    time.sleep(1.0)  # let the loopback connection settle

    table = nt.getTable("SmartDashboard")
    target_speed_sub = table.getDoubleTopic("Swerve/TargetSpeed").subscribe(0.0)
    target_angle_sub = table.getDoubleTopic("Swerve/TargetAngle").subscribe(0.0)
    gyro_pub = table.getDoubleTopic("SimSensors/GyroHeading").publish()
    intake_contact_pub = table.getDoubleTopic("SimSensors/IntakeContact").publish()
    align_time_sub = table.getDoubleTopic("Vision/MetricAlignmentTime").subscribe(-1.0)

    print("[ISAAC_BRIDGE] Core bridge established. Running simulation steps...")

    # Placeholder ground-truth heading state (degrees). Replaced by a real chassis-prim read once a
    # USD robot is loaded.
    heading_deg = 0.0
    dt = 0.02

    step = 0
    metric = -1.0
    while step < args.max_steps:
        if physx_available and not simulation_app.is_running():
            break

        omega_rad_s = target_speed_sub.get()  # commanded yaw rate from the robot
        _goal_deg = target_angle_sub.get()  # available for a full inverse-kinematics mapping

        # === PhysX joint-drive mapping (PLACEHOLDER) ============================================
        # TODO: apply omega_rad_s (and per-wheel translation) to the USD chassis joint drives, then
        # read the resulting chassis yaw back out. Until then, integrate the command as ground truth
        # so the co-sim loop is exercisable:
        heading_deg = (heading_deg + math.degrees(omega_rad_s) * dt + 180.0) % 360.0 - 180.0
        # =======================================================================================

        gyro_pub.set(heading_deg)
        # Intake contact ground truth (did the gripper touch a game piece). PLACEHOLDER: no game
        # piece in the scene yet, so report no contact. Replace with a PhysX contact-report read.
        intake_contact_pub.set(0.0)
        nt.flush()

        final_time = align_time_sub.get()
        if final_time > 0.0:
            metric = final_time
            # The WPILib JVM prints the authoritative [AI_METRIC] line (it knows success/fail); the
            # bridge only logs an info line so it never competes with the orchestrator's grep.
            print(f"[ISAAC_BRIDGE] Robot reported alignment metric: {metric:.2f}s")
            break

        if physx_available:
            sim_context.step(render=args.gui)
        else:
            time.sleep(dt)
        step += 1

    if physx_available:
        simulation_app.close()
    print("[ISAAC_BRIDGE] Simulation execution sequence closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
