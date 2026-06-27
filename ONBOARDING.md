# How this robot project is wired (2027 · Commands v3 · OpModes)

New to this template? Read this once. The biggest surprise for anyone who has seen FRC
code before: **there is no `RobotContainer`.** This template uses WPILib's *OpMode*
framework — the same idea FTC uses. The subsystems and commands underneath are normal
Commands v3; only the top-level *wiring* is different.

> Why this way? It's the structure WPILib's own Commands v3 template uses for 2027, and it
> lets FTC and FRC code share one shape. See `memory`/the team design notes for the full
> rationale. Trade-off: most online FRC tutorials still show `RobotContainer` — this file
> is the translation guide.

## The one-paragraph mental model

- **`Robot.java`** owns the hardware (the subsystems) and runs the command scheduler. That's it.
- Each **"mode"** — driving, an autonomous routine, a calibration task — is its **own class**
  in `opmodes/`, tagged `@Teleop`, `@Autonomous`, or `@Utility`.
- The **driver station lists those classes by name**. Selecting one *constructs* it; that's
  when its button bindings / routine get set up. No `RobotContainer`, no `SendableChooser`.

## If you know `RobotContainer`, here's the map

| Old way (`RobotContainer` + `TimedRobot`) | This template (OpMode) |
| --- | --- |
| Subsystems as fields in `RobotContainer` | `public final` fields on `Robot` |
| `configureBindings()` | each OpMode's **constructor** |
| `getAutonomousCommand()` + `SendableChooser` | one **`@Autonomous` class per routine** |
| `teleopInit()` | a **`@Teleop` class** |
| `testInit()` | a **`@Utility` class** |
| `robotPeriodic()` runs `CommandScheduler` | `Robot.robotPeriodic()` runs `Scheduler` |

## Lifecycle of an OpMode

```
Driver selects it on the DS ─► constructor runs      (build bindings / the routine here)
   │
   ├─ robot disabled & selected ─► disabledPeriodic()  (rarely needed)
   │
   ├─ robot ENABLED ───────────► start()  (once)  ─► periodic()  (~every 20 ms)
   │
   └─ disabled OR another mode picked ─► end()  ─► close()   (object thrown away)

Meanwhile, every loop no matter what: Robot.robotPeriodic() runs Scheduler.getDefault().run()
```

## The one subtle concept: binding scope

A `Trigger` (a button binding) is automatically **scoped to wherever you create it**:

- Created **inside an OpMode constructor** → scoped to that OpMode → **automatically removed
  when you leave the mode.** This is why you never write cleanup code for bindings.
- Created **inside `Robot`'s constructor** (no OpMode selected yet) → **global** → always
  active. Use this only for things like the brake-while-disabled binding.

**Rule of thumb:** per-mode bindings go in the OpMode; always-on bindings go in `Robot`.

## Where things go

| Thing | Where it lives |
| --- | --- |
| Subsystem hardware (motors, sensors) | `public final` fields on `Robot` |
| Controller + button bindings | the `@Teleop` OpMode constructor |
| The joystick *default* drive command | the `@Teleop` OpMode constructor (it needs the controller) |
| An autonomous routine | an `@Autonomous` class — `schedule(...)` in `start()`, `cancel(...)` in `end()` |
| A calibration / transport task | a `@Utility` class |
| Always-on (e.g. brake while disabled) | the `Robot` constructor |

A mechanism with nothing else commanding it automatically holds its **idle** default command
(set up by `Mechanism`), so you don't need to write an explicit "stop."

## "My OpMode doesn't show up on the driver station!"

OpModes are discovered by **scanning classes at runtime**, so a mistake here is **not a
compile error** — the mode just silently doesn't appear. Check, in order:

1. Is the class **`public`** and **not `abstract`**?
2. Is it annotated **`@Teleop` / `@Autonomous` / `@Utility`** with a `name = "..."`?
3. Is it in **`frc.robot`** or a subpackage (we use `frc.robot.opmodes`)?
4. Does it have a **public constructor** taking `(Robot robot)` (or no arguments)?
5. **Read the driver station console.** Selecting a mode prints
   `********** Starting OpMode <name> **********`, so a missing one is easy to spot.

## Hooks we deliberately left out (keep starter OpModes simple)

The framework also offers `addPeriodic`, a custom `getPeriod`, `driverStationConnected`,
`nonePeriodic`, `disabledPeriodic`, watchdog timing, etc. You don't need any of them to start
— ignore them until you have a specific reason.

## Heads-up: this is alpha software

WPILib 2027 is in **alpha** and the OpMode API is still changing (for example, the
`UserControls` annotation was removed upstream — **don't use it**; each OpMode builds its own
controller). This project is pinned to `2027.0.0-alpha-6`. Official docs are still in
progress, so **this file and the comments in the code are your reference.**
