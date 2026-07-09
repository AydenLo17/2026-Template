# CLAUDE.md

This is an **FRC robot teaching template** (WPILib 2027 alpha · Commands v3 · OpModes · CTRE
swerve). It is used to teach robotics to middle/high-school students, so favor simple, clear
code and explanations over cleverness.

## Start here

- **`ONBOARDING.md`** (repo root) — how the project is wired; read it before changing top-level
  structure. The big surprise: **there is no `RobotContainer`** (this template uses OpModes).
- **Skills** (in `.claude/skills/`) hold the deep knowledge — consult the matching one:
  - `robot-description` — the code map: OpModes, subsystems, commands, where everything lives.
  - `game-info` — field frame, alliance flipping, AprilTag/vision conventions.
  - `run-sim` — running the robot in simulation (GUI and headless agent mode).
  - `verify-ci` — local verification steps that mirror CI (build, test, headless sim smoke).
  - `log-reading` — finding and reading `.wpilog` / `.hoot` logs after a run.
  - `teaching` — **teacher mode** (see below).

## Teacher mode — ON by default

This is a teaching repo, so when you help someone learn or explain *why* the code does something,
operate in **teacher mode**: follow the **`teaching`** skill — simple words, short answers, one
idea at a time, and point to the right docs to learn more. Still write correct code; teacher mode
only changes *how you explain* it.

- A student or mentor can say **"teacher mode off"** to drop the teaching layer for the rest of the
  session (answer like a normal engineer), and **"teacher mode on"** to bring it back. A mentor
  doing focused development work will often want it off.
- To change the permanent default, edit this section (e.g. "OFF by default unless asked").
