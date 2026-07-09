---
name: verify-ci
description: Run the repo's reliability verification loop locally the same way CI does: build, test, and headless simulateJavaAgent smoke check. Use this before opening a PR, after major command/subsystem changes, or when debugging CI failures.
---

# Verify CI flow locally

Use this skill when you want a fast, repeatable "are we shippable?" check.

This skill is guidance for agents and humans.
The enforcing gate is still [.github/workflows/ci.yml](.github/workflows/ci.yml).

## What this repo's CI verifies

The CI workflow runs three checks:

1. Build
2. Unit tests
3. Headless simulation smoke run

Local equivalents:

```powershell
./gradlew --no-daemon build
./gradlew --no-daemon test
./gradlew --no-daemon simulateJavaAgent -Pmode=auto
```

## Recommended order while developing

Run these in order for quicker feedback:

```powershell
# fastest signal first
./gradlew --no-daemon test

# full compile/package check
./gradlew --no-daemon build

# runtime smoke check in headless autonomous mode
./gradlew --no-daemon simulateJavaAgent -Pmode=auto
```

If `simulateJavaAgent` keeps running, stop it after you see startup + autonomous begin.
On Windows terminal, use `Ctrl+Break` so logs flush cleanly.

## Expected success signals

- `test`: all tests pass, Gradle exits 0
- `build`: build successful, Gradle exits 0
- `simulateJavaAgent`: console shows robot startup, then an autonomous OpMode starts:

```text
********** Robot program startup complete **********
[SimStartup] Headless start: enabled=true mode=AUTONOMOUS opmode="..."
********** Starting OpMode ... **********
```

## Quick triage when CI fails

- Fails in `test`:
  - Re-run `./gradlew --no-daemon test`
  - Fix deterministic test failures first
- Fails in `build`:
  - Re-run `./gradlew --no-daemon build`
  - Check compile or formatting-related output
- Fails in simulation smoke:
  - Re-run `./gradlew --no-daemon simulateJavaAgent -Pmode=auto`
  - Verify `[SimStartup]` line appears and an OpMode starts

## Notes specific to this template

- Uses WPILib 2027 alpha + Java 25; use the WPILib toolchain JDK.
- Headless sim behavior is wired in [build.gradle](build.gradle) and [src/main/java/frc/robot/utils/SimStartup.java](src/main/java/frc/robot/utils/SimStartup.java).
- This is not a replacement for CI; it is the local playbook to match CI behavior.
