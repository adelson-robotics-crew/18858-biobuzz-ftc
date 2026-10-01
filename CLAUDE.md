# Code style

- Always make variable names easy to read and understand. Opt for longer, more descriptive names over one-letter or abbreviated names. For example, use `poseFactory` instead of `p`.
- Always include a docstring (Javadoc) for every function.
- For ambiguous lines of code, always add a comment explaining what they do.

# Architecture

- Each subsystem class owns its own hardware. Nothing outside it touches those objects directly. A subsystem can also have its own internal state if it needs one.
- `Robot.java` owns all subsystems, is the single hardware init point, and holds the logic that ties them together (including gamepad handling and which mechanisms may run together).
- `RobotConstants.java` is definitions only, with no logic. It has clearly labeled sections for hardware names, gamepad button mapping, and tunable robot constants. Pedro's own tuned values stay in `subsystems/drivetrain/pedro/Constants.java`.

# Command-based (Ivy)

- **Default stays enum + switch.** Every subsystem defaults to a plain per-subsystem FSM (a `State` enum plus a switch in `update()`). Ivy commands are not the new default. Use them only when a case earns it.
- **Use Ivy commands only when one of these is concretely true:**
  - Multi-step sequencing where steps run in order and each step's completion condition differs. The reference example is the autonomous leg-by-leg movement, which is why we adopted Ivy.
  - A real need for parallel or race execution (two things at once, or "whichever finishes first wins"). A switch can't express this cleanly.
  - Cross-subsystem mutual exclusion that would otherwise need `isActive()` glue in `Robot`. Intake/Shooter still use hand-written checks today. Migrate them only if the check grows past a simple two-way check.
  - Reusable pieces you want to compose differently across multiple autonomous routines.
- **Current scope:** the Scheduler is wired into the autonomous OpMode's lifecycle (`Scheduler.reset()` in `init()`, `Scheduler.execute()` in `loop()`), and the autonomous routine is a command composition. Intake, Shooter, and TeleOp stay plain enum + switch. Don't migrate a subsystem unless a trigger above is true for that subsystem specifically. "For consistency" is not a reason.
- **Any OpMode that calls `Scheduler.execute()` must call `Scheduler.reset()` in `init()`.** The Scheduler is static and outlives OpModes, so leftover commands would run against the previous OpMode's hardware.
- **Ownership still applies:** the Pedro `Follower` stays private inside `Drivetrain`. Code that needs follow/hold behavior gets it from `Drivetrain`'s command-returning methods (`followPathCommand()`, `holdPoseCommand()`) and never uses `PedroCommands` with the Follower directly. Those methods finish on `isAtPose()`, not on Ivy's defaults: Ivy's `follow` ends at `atParametricEnd()` and its `hold` is instant, and both end before the robot has settled.
