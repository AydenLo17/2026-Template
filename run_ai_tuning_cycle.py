"""Headless AI tuning orchestrator for the VisionAutoAlign gains.

This script runs the reliability loop described in ``ISAAC_SIM_AUTOMATION.md``:

1. Launch the headless WPILib simulation on this repo's real task
   (``gradlew simulateJavaAgent -Pmode=auto:Vision Auto Align``), which auto-enables the robot in
   the "Vision Auto Align" autonomous OpMode.
2. Optionally launch the Isaac Sim physics bridge (``isaac_sim_bridge.py``) if the
   ``ISAAC_SIM_PYTHON`` environment variable points at an Isaac Sim python wrapper. Without it, the
   WPILib/CTRE simulation provides its own heading physics and the loop still runs.
3. Read the child processes' stdout, capture the ``[AI_METRIC]`` line the robot prints, and extract
   the alignment duration.
4. Terminate the processes cleanly so the loopback NetworkTables ports are freed.

Two modes:

* Default: run a **single** iteration and print the captured metric. This matches the doc's
  single-shot script and is what a human-in-the-loop agent (Claude) uses -- it edits the gains in
  ``Constants.java`` between the ``AI_TUNABLE_ZONE`` markers, reruns this, and reads the number.
* ``--iterations N``: run a small **automated** coordinate-search over kP/kD, rewriting the tunable
  zone between runs, stopping early once alignment converges under ``--target`` seconds.

Nothing here mutates anything outside the two ``AI_TUNABLE_ZONE`` markers in ``Constants.java``.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path
from queue import Empty, Queue

REPO = Path(__file__).resolve().parent
CONSTANTS = REPO / "src" / "main" / "java" / "frc" / "robot" / "Constants.java"
BRIDGE = REPO / "isaac_sim_bridge.py"

START_MARKER = "// AI_TUNABLE_ZONE_START"
END_MARKER = "// AI_TUNABLE_ZONE_END"

# Matches: [AI_METRIC] AlignmentTime: 1.23s | Success: True
METRIC_RE = re.compile(
    r"\[AI_METRIC\]\s*AlignmentTime:\s*([0-9.]+)s\s*\|\s*Success:\s*(True|False)"
)

# The console line SimStartup prints once the headless sim auto-enables the OpMode.
ENABLED_MARKER = "SimStartup] Headless start"

# The OpMode name registered by OpModes.VisionAutoAlignAuto.
OPMODE = "Vision Auto Align"


# --------------------------------------------------------------------------------------------- I/O
def _gradlew() -> str:
    return str(REPO / ("gradlew.bat" if os.name == "nt" else "gradlew"))


def _spawn(cmd, cwd) -> subprocess.Popen:
    """Start a child process in its own group so we can kill the whole tree later."""
    kwargs = dict(
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    if os.name == "nt":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]
    else:
        kwargs["start_new_session"] = True
    return subprocess.Popen(cmd, **kwargs)


def _kill(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        if os.name == "nt":
            subprocess.run(
                ["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        else:
            import signal

            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=10)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def _pump(proc: subprocess.Popen, tag: str, q: Queue) -> threading.Thread:
    """Stream a process's stdout to our console (prefixed) and into a queue for scanning."""

    def run():
        assert proc.stdout is not None
        for line in proc.stdout:
            line = line.rstrip("\n")
            print(f"[{tag}] {line}")
            q.put(line)
        q.put(None)  # sentinel: stream closed

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


# --------------------------------------------------------------------------- Constants.java editing
def read_gains() -> dict[str, float]:
    text = CONSTANTS.read_text(encoding="utf-8")
    zone = text.split(START_MARKER, 1)[1].split(END_MARKER, 1)[0]
    gains = {}
    for key in ("kP", "kI", "kD"):
        m = re.search(rf"{key}\s*=\s*([0-9.eE+-]+)\s*;", zone)
        if m:
            gains[key] = float(m.group(1))
    return gains


def write_gains(kp: float, ki: float, kd: float) -> None:
    """Rewrite only the kP/kI/kD assignment lines inside the AI_TUNABLE_ZONE.

    Targeted line edits (not a whole-zone rewrite) so the other tunable fields in the zone -- the
    vision latency compensation and intake current/speed -- are preserved untouched.
    """
    text = CONSTANTS.read_text(encoding="utf-8")
    head, rest = text.split(START_MARKER, 1)
    zone, tail = rest.split(END_MARKER, 1)
    zone = re.sub(r"(kP\s*=\s*)[0-9.eE+-]+", rf"\g<1>{kp:.5f}", zone, count=1)
    zone = re.sub(r"(kI\s*=\s*)[0-9.eE+-]+", rf"\g<1>{ki:.5f}", zone, count=1)
    zone = re.sub(r"(kD\s*=\s*)[0-9.eE+-]+", rf"\g<1>{kd:.5f}", zone, count=1)
    CONSTANTS.write_text(head + START_MARKER + zone + END_MARKER + tail, encoding="utf-8")


# ------------------------------------------------------------------------------------- one run cycle
def run_iteration(run_window: float = 15.0, boot_timeout: float = 180.0) -> tuple[bool, float] | None:
    """Compile + run the headless sim (and optional Isaac bridge). Returns (success, seconds).

    The directive asks for a fixed run duration (~15 s). Because a cold ``gradlew`` build plus JVM
    and sim boot can take much longer than that, the fixed window is applied to the *sim run itself*
    -- it starts once ``SimStartup`` reports the robot enabled -- while ``boot_timeout`` covers the
    build/boot phase before that. Either way the loop exits early the instant the metric appears.
    """
    print("[ORCHESTRATOR] Initializing execution cycle...")
    q: Queue = Queue()

    wpilib_cmd = [_gradlew(), "simulateJavaAgent", f"-Pmode=auto:{OPMODE}"]
    wpilib = _spawn(wpilib_cmd, REPO)
    _pump(wpilib, "WPILIB", q)

    # Optional Isaac Sim bridge. Only launched when an Isaac python wrapper is configured.
    isaac = None
    isaac_python = os.environ.get("ISAAC_SIM_PYTHON")
    if isaac_python and Path(isaac_python).exists() and BRIDGE.exists():
        print(f"[ORCHESTRATOR] Launching Isaac bridge via {isaac_python}")
        time.sleep(3.0)  # let the JVM NetworkTables server come up first
        isaac = _spawn([isaac_python, str(BRIDGE)], REPO)
        _pump(isaac, "ISAAC", q)
    else:
        print("[ORCHESTRATOR] ISAAC_SIM_PYTHON not set/found -> WPILib-only physics (still valid).")

    result: tuple[bool, float] | None = None
    enabled_at: float | None = None
    boot_deadline = time.time() + boot_timeout
    try:
        while True:
            now = time.time()
            if enabled_at is None and now > boot_deadline:
                print("[ORCHESTRATOR] Timed out waiting for the sim to enable.")
                break
            if enabled_at is not None and now > enabled_at + run_window:
                print(f"[ORCHESTRATOR] Fixed run window ({run_window:.0f}s) elapsed with no metric.")
                break
            if wpilib.poll() is not None and q.empty():
                break
            try:
                line = q.get(timeout=0.5)
            except Empty:
                continue
            if line is None:
                continue
            if enabled_at is None and ENABLED_MARKER in line:
                enabled_at = time.time()
            m = METRIC_RE.search(line)
            if m:
                result = (m.group(2) == "True", float(m.group(1)))
                break
    finally:
        # Kill both process trees so the loopback NetworkTables ports (5810 NT4, 5800 Limelight)
        # are freed before the next iteration.
        _kill(isaac)
        _kill(wpilib)

    print("[ORCHESTRATOR] Run cycle finalized. Loopback ports cleared.")
    return result


# ----------------------------------------------------------------------------------- automated tune
def auto_tune(iterations: int, target: float) -> None:
    """A small coordinate search: push kP up while it helps, add kD if a run fails to converge."""
    gains = read_gains()
    kp = gains.get("kP", 0.05)
    ki = gains.get("kI", 0.0)
    kd = gains.get("kD", 0.001)
    best = None  # (time, kp, ki, kd)

    for i in range(1, iterations + 1):
        write_gains(kp, ki, kd)
        print(f"\n===== Iteration {i}/{iterations}: kP={kp:.5f} kI={ki:.5f} kD={kd:.5f} =====")
        res = run_iteration()
        if res is None:
            print("[TUNER] No metric captured (build/run problem) -> stopping.")
            return

        success, seconds = res
        print(f"[TUNER] success={success} time={seconds:.2f}s")

        if success:
            if best is None or seconds < best[0]:
                best = (seconds, kp, ki, kd)
            if seconds <= target:
                print(f"[TUNER] Target met ({seconds:.2f}s <= {target:.2f}s). Optimized.")
                break
            # Converged but slow: raise kP to close faster; nudge kD to keep it damped.
            kp *= 1.4
            kd += 0.0005
        else:  # Failed to converge: likely oscillating past tolerance -> ease kP, add damping.
            kp *= 0.6
            kd += 0.001

    if best is not None:
        print(f"\n[TUNER] Best converged run: {best[0]:.2f}s at kP={best[1]:.5f} kI={best[2]:.5f} kD={best[3]:.5f}")
        write_gains(best[1], best[2], best[3])
        print("[TUNER] Wrote best gains back to Constants.java.")


# ------------------------------------------------------------------------------------------- CLI
def main() -> int:
    parser = argparse.ArgumentParser(description="Headless AI tuning loop for VisionAutoAlign")
    parser.add_argument("--iterations", type=int, default=1, help="Automated tuning iterations")
    parser.add_argument("--target", type=float, default=1.25, help="Target alignment time (s)")
    parser.add_argument(
        "--run-window", type=float, default=15.0, help="Fixed sim run window after enable (s)"
    )
    args = parser.parse_args()

    if not CONSTANTS.exists():
        print(f"[ORCHESTRATOR] Cannot find {CONSTANTS}", file=sys.stderr)
        return 1

    if args.iterations > 1:
        auto_tune(args.iterations, args.target)
        return 0

    res = run_iteration(run_window=args.run_window)
    if res is None:
        print("[ORCHESTRATOR] No [AI_METRIC] captured this run.")
        return 1
    success, seconds = res
    print(f"\nALIGNMENT_SUCCESS={success}")
    print(f"ALIGNMENT_TIME={seconds:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
