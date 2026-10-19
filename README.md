# A Package by FRC team 1209

[![Auto Create Release](https://github.com/robohornets/WhatTime/actions/workflows/auto-release.yml/badge.svg)](https://github.com/robohornets/WhatTime/actions/workflows/auto-release.yml) 
[![pages-build-deployment](https://github.com/robohornets/WhatTime/actions/workflows/pages/pages-build-deployment/badge.svg?branch=gh-pages)](https://github.com/robohornets/WhatTime/actions/workflows/pages/pages-build-deployment) 
![GitHub Release](https://img.shields.io/github/v/release/robohornets/WhatTime)

WhatTime is a set of helpers for FRC robots that use TalonFX motors:

- **[Motor](#motor)**: drive a TalonFX at a speed, or move it to a position and hold it there, with soft limits, acceleration and turret wraparound.
- **[MotorGroup](#motorgroup)**: control several motors together.
- **[MatchTimeManager](#matchtimemanager)**: run commands at set times in teleop.
- **[RebuiltHubManager](#rebuilthubmanager)**: track whether your hub is active in 2026 REBUILT.
- **[Dashboard helpers](#dashboard-helpers)**: one-line NetworkTables and SmartDashboard publishing.

Older documentation lives at [readthedocs.1209robohornets.com](https://readthedocs.1209robohornets.com/latest/), but it hasn't been updated for the 2026.2 refactor yet. This README covers the current version.

## Installation

In VS Code, open the command palette and run **WPILib: Manage Vendor Libraries → Install new libraries (online)**, then paste:

```
https://raw.githubusercontent.com/robohornets/WhatTime/refs/heads/main/WhatTime.json
```

WhatTime also needs the Phoenix 6 vendor library.

## Motor

`Motor` wraps a TalonFX. It is a WPILib subsystem that updates itself every loop, so a single call is enough. `drive(0.5)` keeps the motor spinning until you call `stop()`. `goTo(30)` moves it to 30 and holds it there. You never need to call anything every loop.

```java
import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);

    private final Motor intake = new Motor(10);
    private final Motor elevator = new Motor(11)
        .setRange(0, 50)
        .setMotorSpeed(0.6)
        .setAccelerationTime(0.3)
        .setHoldSpeed(0.05);

    public RobotContainer() {
        controller.a().whileTrue(intake.driveCommand(0.8)); // runs while held, stops on release
        controller.y().onTrue(elevator.goToCommand(40));    // moves up to 40 and holds
        controller.x().onTrue(elevator.goToCommand(0));     // moves back down
    }
}
```

The motor always stops when the robot is disabled, and it starts from a stop when the robot is enabled again.

### Creating a motor

```java
new Motor(10);                  // TalonFX with CAN id 10
new Motor(10, true);            // inverted
new Motor(10, "canivore");      // on a CANivore bus
new Motor(10, "canivore", true);
Motor.of(existingTalonFX, false);
```

### Driving

| Method | What it does |
| --- | --- |
| `drive()` | Drives forward at `motorSpeed` until stopped. |
| `drive(boolean reverse)` | Drives at `motorSpeed`, backwards if `reverse` is true. |
| `drive(double speed)` | Drives at `speed` (-1.0 to 1.0). |
| `stop()` | Stops and clears any position target. |
| `driveCommand(double speed)` | A command that drives while it runs and stops when it ends. Use it with `whileTrue`. |

### Moving to a position

`goTo` drives toward the target at `kP * distance` and caps that at `motorSpeed`, or at `motorUpSpeed`/`motorDownSpeed` if you set them. Once the motor is within `threshold` of the target, it switches to `holdSpeed`. It only starts correcting again after drifting `2 × threshold` away, so it doesn't jitter at the edge.

| Method | What it does |
| --- | --- |
| `goTo(double position)` | Moves to `position` and holds it until told to do something else. |
| `setTarget(double position)` | Same as `goTo`. |
| `isAtTarget()` | True when there's a target and the motor is within `threshold` of it. |
| `goToCommand(double position)` | A command that moves to `position` and finishes once it's there. The motor keeps holding afterwards, so you can chain commands: `elevator.goToCommand(30).andThen(score)`. |
| `getPosition()` | The current position. It comes from the position supplier if you set one, otherwise from the TalonFX encoder in motor rotations. |

### Acceleration

```java
new Motor(11).setAccelerationTime(0.5); // 0.5s from stopped to full speed
```

With an acceleration time set, the motor changes speed gradually. That applies to speeding up, slowing down, stopping and reversing (which slows to zero before going the other way). It's off (`0`) by default. The one exception is the robot being disabled: the robot cuts motor power immediately no matter what.

Because stopping is gradual too, a motor that reaches a soft limit only starts slowing down there, so leave some room before any hard stop.

### Soft limits

```java
new Motor(11).setRange(0, 50); // or setMinValue(0) / setMaxValue(50)
```

The motor won't drive further past a limit in either `drive` or `goTo` mode, and `goTo` targets are clamped into the range. There are no limits by default.

### Turrets and other continuous mechanisms

```java
Motor turret = new Motor(12).setContinuous(0, 1);
turret.goTo(0.1); // from 0.9 this moves forward 0.2, not back 0.8
```

`setContinuous(min, max)` treats `min` and `max` as the same spot, so `goTo` always takes the shortest way around. This works no matter how many times the mechanism has spun. Use the length of one full turn in your position units. With the built-in encoder, that's motor rotations, so a turret on a 10:1 gearbox would use `setContinuous(0, 10)`. Don't combine this with soft limits, because the shortest way may cross them.

### All settings

Every setter returns the motor, so you can chain them.

| Setter | Default | What it does |
| --- | --- | --- |
| `setInverted(boolean)` | `false` | Flips the direction. The built-in encoder is flipped to match. |
| `setRange(min, max)` | no limits | Soft limits (see above). `setMinValue` and `setMaxValue` set one side. |
| `setContinuous(min, max)` | off | Shortest-way-around for turrets (see above). |
| `setMotorSpeed(double)` | `1.0` | Speed for `drive()`, and the top speed for `goTo`. |
| `setMotorUpSpeed(Double)` | `motorSpeed` | Top `goTo` speed toward higher positions. `null` falls back to `motorSpeed`. |
| `setMotorDownSpeed(Double)` | `motorSpeed` | Top `goTo` speed toward lower positions. `null` falls back to `motorSpeed`. |
| `setMinSpeed(double)` | `0.0` | Smallest non-zero speed, so the motor doesn't stall on friction near a target. |
| `setAccelerationTime(double)` | `0.0` (off) | Seconds from stopped to full speed (see above). |
| `setKP(double)` | `0.1` | How hard `goTo` pushes: speed = kP × distance. Higher is faster but can overshoot. |
| `setThreshold(double)` | `0.025` | How close to the target counts as being there. |
| `setHoldSpeed(double)` | `0.0` | Output while holding at the target, e.g. a little upward power for an elevator. |
| `setPositionSupplier(DoubleSupplier)` | TalonFX encoder | Read the position from another sensor, e.g. a CANcoder. Its value is used as-is. |
| `setEnabled(boolean)` | `true` | Disabling stops the motor and ignores `drive`/`goTo` until it's enabled again. `toggleEnabled()` and `isEnabled()` also exist. |
| `setNeutralMode(NeutralModeValue)` | TalonFX setting | `Brake` resists turning when there's no power; `Coast` spins freely. |

### Other methods

| Method | What it does |
| --- | --- |
| `brakelessReset(double seconds)` | A command that stops the motor and coasts for `seconds`, e.g. so a mechanism can settle against a hard stop, then switches back to brake mode. |
| `getMotor()` / `getTalonFX()` | The underlying TalonFX, for anything `Motor` doesn't cover, like current limits. Don't call `set()` on it yourself, because `Motor` overwrites the output every loop. |
| `isInverted()` | Whether the motor is inverted. |

## MotorGroup

`MotorGroup` controls several motors together, for example the two motors on an elevator. It has the same methods as `Motor`, and each one applies to every motor in the group.

```java
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorGroup;

MotorGroup elevator = new MotorGroup(new Motor(11), new Motor(12, true))
    .setRange(0, 50)
    .setAccelerationTime(0.3);

controller.y().onTrue(elevator.goToCommand(40)); // finishes once every motor is there
```

Each motor uses its own position. If one motor is mounted the other way, create it with `inverted = true` so both count up in the same direction. Alternatively, give the group one shared sensor with `setPositionSupplier`. `group.setInverted(...)` sets every motor to the same value. `getMotors()` returns the motors in the group.

## MatchTimeManager

`MatchTimeManager` runs a command when a certain number of seconds are left in teleop.

```java
import com.btwrobotics.WhatTime.frc.DriverStation.MatchTimeManager;

MatchTimeManager matchTime = new MatchTimeManager();
matchTime.scheduleEventAtTime(30, rumbleCommand(), "Endgame rumble");
```

Events only fire during teleop. Pass a new command each time, since WPILib doesn't allow one command to be scheduled in two places. `pendingTriggerDescriptions` and `completedTriggerDescriptions` list the events that haven't run yet and those that have.

## RebuiltHubManager

`RebuiltHubManager` tracks whether your alliance's hub is active during a 2026 REBUILT match. Both hubs are active in auto, the transition shift and endgame. In between, the hubs take turns, starting with the alliance named in the game-specific message.

```java
import com.btwrobotics.WhatTime.frc.YearlyMethods.Rebuilt.RebuiltHubManager;

RebuiltHubManager hub = new RebuiltHubManager(matchTime);
hub.scheduleAllInactiveHubChanges(); // call once, e.g. in RobotContainer

new Trigger(hub::hubIsActive).whileTrue(shootCommand());
```

| Method | What it does |
| --- | --- |
| `scheduleAllInactiveHubChanges()` | Schedules every phase change. Call it once. |
| `hubIsActive()` | Whether your hub is active right now. |
| `getInactiveFirstAlliance()` | Which alliance's hub goes inactive first, once the game data is known. |
| `updateInitialInactiveAlliance()` | Reads that from the game-specific message. This happens automatically at the start of phase 1, so you only need it if you want the value earlier. |
| `putPhasesToNetworkTables()` | Publishes the upcoming and past phases to NetworkTables. You can also pass a table name. |

`AllianceManager` has the small helpers this uses: `getCurrentAlliance()`, `getOpposingAlliance()` and `toggleAlliance(alliance)`.

## Dashboard helpers

```java
import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;
import com.btwrobotics.WhatTime.frc.DashboardManagers.ShuffleboardUtil;

NetworkTablesUtil.put("Elevator Height", elevator.getPosition());   // "CustomDashboard" table
NetworkTablesUtil.put("Vision", "Has Target", true);                // a table of your choice
NetworkTable table = NetworkTablesUtil.getTable("Vision");

ShuffleboardUtil.put("Hub Active", hub.hubIsActive());              // SmartDashboard
```

Both take numbers, strings and booleans. Any other object is published as its `toString()`.

## Maintainers

*   **Tullysaurus** (Python maintainer)
*   **CaedmonMyers** (Java maintainer)
