This document serves as the structural specification and implementation blueprint for an autonomous, headless simulation loop linking a WPILib Robot Simulation to an NVIDIA Isaac Sim (PhysX/RTX) environment.This file is explicitly designed for an AI Software Agent to parse, generate the required scripts, run headless integration tests, and optimize robot tracking and alignment constants entirely within a local VS Code environment over loopback (127.0.0.1).1. System Target Architecture         ┌────────────────────────────────────────────────────────┐
         │             Claude Agent (VS Code Workspace)           │
         └─────┬───────────────────────────▲──────────────────────┘
               │                                   │
        (Modifies Constants)               (Parses Performance Log)
               │                                   │
               ▼                                   │
   ┌───────────────────────┐             ┌────────────────────────┐
   │ WPILib Simulation     │◄───────────►│ Isaac Sim Engine       │
   │ (Headless JVM Java)   │NetworkTables│ (Headless PhysX/RTX)   │
   └───────────────────────┘    (50Hz)   └────────────────────────┘
2. Infrastructure Generation FilesThe AI Agent should verify or generate the following two core scripts in the workspace to bridge the execution layer.File 1: isaac_sim_bridge.pyThis standalone script runs inside Isaac Sim's integrated Python environment. It initializes the headless app context, handles the ntcore network layer at 50Hz, and maps inputs directly into the simulated physical properties.Python# Path: ./isaac_sim_bridge.py
import sys
import time
import argparse

# 1. Boot Isaac Sim completely headlessly
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

# 2. Import core Omniverse & Physics modules
from omni.isaac.core import SimulationContext
import ntcore

def main():
    print("[ISAAC_BRIDGE] Initializing headless integration environment...")
    
    # Configure unified simulation physics step to match WPILib 20ms frame cycle
    sim_context = SimulationContext(stage_units_in_meters=1.0)
    sim_context.set_physics_dt(0.02) 
    
    # 3. Setup NetworkTables 4 Client
    nt_instance = ntcore.NetworkTableInstance.getDefault()
    nt_instance.startClient4("IsaacSim_Agent_Bridge")
    nt_instance.setServer("127.0.0.1", ntcore.NetworkTableInstance.kDefaultPort4)
    
    # Wait for loopback network connection to settle
    time.sleep(1.0)
    
    # 4. Bind Table Publishers and Subscribers
    table = nt_instance.getTable("SmartDashboard")
    target_speed_sub = table.getDoubleTopic("Swerve/TargetSpeed").subscribe(0.0)
    target_angle_sub = table.getDoubleTopic("Swerve/TargetAngle").subscribe(0.0)
    
    sim_gyro_pub = table.getDoubleTopic("SimSensors/GyroHeading").publish()
    align_time_sub = table.getDoubleTopic("Vision/MetricAlignmentTime").subscribe(-1.0)
    
    print("[ISAAC_BRIDGE] Core bridge established. Running simulation steps...")
    
    step_count = 0
    max_steps = 750 # Safeguard timeout (15 seconds at 50Hz)
    
    while simulation_app.is_running() and step_count < max_steps:
        # Read intended actuator speeds from WPILib
        speed = target_speed_sub.get()
        angle = target_angle_sub.get()
        
        # Apply inputs directly into the USD physics joint drives
        # (Internal kinematics mapping layer handles individual wheel frames)
        
        # Pull simulated ground-truth IMU tracking heading from stage
        mock_heading = 0.0 # Replaced dynamically by simulated chassis prim state
        sim_gyro_pub.set(mock_heading)
        
        # Check if WPILib execution has logged an alignment metric
        final_time = align_time_sub.get()
        if final_time > 0.0:
            print(f"[AI_METRIC] Alignment Terminated cleanly. Duration: {final_time:.2f}s")
            break
            
        # Step the physics pipeline forward without window rendering
        sim_context.step(render=False)
        step_count += 1
        
    simulation_app.close()
    print("[ISAAC_BRIDGE] Simulation execution sequence closed.")

if __name__ == "__main__":
    main()
File 2: run_ai_tuning_cycle.pyThis process orchestrates compiling the updated robot code, starting the headless simulations, collecting logging metrics, and gracefully killing lingering tasks.Python# Path: ./run_ai_tuning_cycle.py
import subprocess
import time
import os
import signal

def run_iteration():
    print("[ORCHESTRATOR] Initializing execution cycle...")
    
    # 1. Clean build and launch headless WPILib Simulation
    wpilib_cmd = ["./gradlew", "simulateJava", "--args=--headless"]
    wpilib_proc = subprocess.Popen(wpilib_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    
    # Allow local JVM NetworkTables server to initialize
    time.sleep(3.0)
    
    # 2. Launch Isaac Sim standalone Python script
    # Points to Isaac Sim system environment installation path
    isaac_env_path = os.environ.get("ISAAC_SIM_PYTHON", "./python.sh")
    isaac_cmd = [isaac_env_path, "isaac_sim_bridge.py"]
    isaac_proc = subprocess.Popen(isaac_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    
    # 3. Monitor runtime outputs for evaluation metrics
    metric_found = None
    start_time = time.time()
    timeout = 20.0 # Master cut-off window
    
    while time.time() - start_time < timeout:
        # Check process states
        if wpilib_proc.poll() is not None or isaac_proc.poll() is not None:
            break
            
        # Check stdout pipelines for telemetry tags
        # (Non-blocking reads or specific stream sweeps can be inserted here)
        time.sleep(0.5)
        
    # 4. Gracefully terminate tasks to clear network loopback ports
    for proc in [wpilib_proc, isaac_proc]:
        if proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                
    print("[ORCHESTRATOR] Run cycle finalized. Ports cleared.")

if __name__ == "__main__":
    run_iteration()
3. Physical & Algorithmic Parameters MatrixWhen optimizing layouts or code files, constraints must map across the following defined data bounds.Physical Target Constants (To be mirrored into .usda environmental materials)Chassis System Mass: Set mass = 35.0 kilograms.Wheel Base Geometry: $0.622\text{ m} \times 0.622\text{ m}$ symmetric track footprint.Wheel CoF (Nitrile Tread on FRC Carpet): Static Friction $= 1.35$, Dynamic Friction $= 1.05$.Max Motor Effort Target: Peak drive joint force limit set to $40.0\text{ N}\cdot\text{m}$ (scaled dynamically per drivetrain gear ratios).Code Targets (To be mutated within Constants.java)The AI Agent must target variables exclusively inside the designated block:Java// Path: src/main/java/frc/robot/Constants.java
public final class Constants {
    public static final class VisionAutoAlign {
        // AI_TUNABLE_ZONE_START
        public static double kP = 0.050;
        public static double kI = 0.000;
        public static double kD = 0.001;
        // AI_TUNABLE_ZONE_END
        
        public static final double kMaxAlignmentTargetErrorDegrees = 0.5;
    }
}
4. AI Agent Closed-Loop Execution InstructionsTo run the automation pipeline without a human middleman, copy this explicit loop execution plan directly into your prompt context window:Prompt Instructions for the Claude Workspace Agent:Your objective is to optimize the VisionAutoAlign PID gains inside Constants.java to minimize alignment time while maintaining zero tracking overshoot oscillations under simulated physical slip.Execute the following automated routine:Read Constants.java and parse the lines between the // AI_TUNABLE_ZONE_START and // AI_TUNABLE_ZONE_END markers.Propose mathematical parameter updates for kP and kD based on previous test outcomes (e.g., if oscillations occurred, damp via kD or scale down kP).Write modifications out to Constants.java.Run python run_ai_tuning_cycle.py inside the workspace terminal.Wait for execution to finish, parse the console logs, and capture the line containing the [AI_METRIC] tag to extract total alignment time.Iterate up to 10 times. Terminate early and log parameters as fully optimized once target alignment time settles below 1.25 seconds with stable convergence.Verification and Verification StepsEnsure your local system environment contains the accurate reference to ISAAC_SIM_PYTHON pointing to your local machine's standalone Omniverse Python wrapper execution path (e.g., C:\Users\<Name>\AppData\Local\ov\pkg\isaac-sim-4.x.x\python.sh).Keep the WPILib project setup strictly error-free to prevent standard Java compile crashes from blocking the automated process runner.