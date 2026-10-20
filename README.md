# A Package by FRC team 1209

[![Auto Create Release](https://github.com/robohornets/WhatTime/actions/workflows/auto-release.yml/badge.svg)](https://github.com/robohornets/WhatTime/actions/workflows/auto-release.yml) 
[![pages-build-deployment](https://github.com/robohornets/WhatTime/actions/workflows/pages/pages-build-deployment/badge.svg?branch=gh-pages)](https://github.com/robohornets/WhatTime/actions/workflows/pages/pages-build-deployment) 
![GitHub Release](https://img.shields.io/github/v/release/robohornets/WhatTime)

WhatTime is a set of helpers for FRC robots that use TalonFX motors:

- **[Motor](#motor)**: drive a TalonFX at a speed, spin it at a velocity, or move it to a position and hold it there. Includes soft limits, acceleration, turret wraparound and rotation limits, Motion Magic, current limits, gear ratios, homing, dashboard telemetry and simulation.
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

`Motor` wraps a TalonFX. It is a WPILib subsystem that updates itself every loop, so a single call is enough. `drive(0.5)` keeps the motor spinning until you call `stop()`. `setVelocity(80)` holds 80 rotations per second. `goTo(30)` moves it to 30 and holds it there. You never need to call anything every loop.

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
| `stop()` | Stops and clears any position or velocity target. |
| `driveCommand(double speed)` | A command that drives while it runs and stops when it ends. Use it with `whileTrue`. |

### Moving to a position

`goTo` drives toward the target at `kP * distance` and caps that at `motorSpeed`, or at `motorUpSpeed`/`motorDownSpeed` if you set them. Once the motor is within `threshold` of the target, it switches to `holdSpeed`. It only starts correcting again after drifting `2 × threshold` away, so it doesn't jitter at the edge.

| Method | What it does |
| --- | --- |
| `goTo(double position)` | Moves to `position` and holds it until told to do something else. |
| `setTarget(double position)` | Same as `goTo`. |
| `isAtTarget()` | True when there's a target and the motor is within `threshold` of it. |
| `goToCommand(double position)` | A command that moves to `position` and finishes once it's there. The motor keeps holding afterwards, so you can chain commands: `elevator.goToCommand(30).andThen(score)`. |
| `getPosition()` | The current position. It comes from the position supplier if you set one, otherwise from the TalonFX's sensor in rotations (mechanism rotations with `setGearRatio`). |

### Motion Magic: running `goTo` on the TalonFX

By default, `goTo` is worked out in robot code 50 times a second. `setMotionMagic` hands it to the TalonFX instead. The TalonFX plans a smooth move that speeds up, cruises and slows down, then follows it 1000 times a second with its own PID and feedforward. That makes it smoother and more accurate, especially for elevators and arms, but the gains need tuning.

```java
Motor elevator = new Motor(11)
    .setGearRatio(12)
    .setRange(0, 4)
    .setMotionMagic(2, 4)          // max 2 rotations/s, speeding up and slowing down at 4 rotations/s²
    .setPID(20, 0, 0.5)            // volts per rotation of error, and so on
    .setFeedforward(0.1, 1.5, 0)   // kS, kV, kA
    .setGravity(0.4, false);       // volts to hold it up; true for an arm

controller.y().onTrue(elevator.goToCommand(3)); // same commands as before
```

- **Gains are in volts:** `setPID(kP, kI, kD)` sets volts per rotation of error. `setFeedforward(kS, kV, kA)` sets volts to overcome friction, volts per rotation per second, and volts per rotation per second². These are separate from `setKP`, which is only for the default `goTo`.
- **`setGravity(kG, arm)`:** adds `kG` volts to hold the mechanism up. For an arm (`true`), it's scaled by the arm's angle. Position 0 must be the arm sticking straight out sideways, and positions must be in arm rotations (use `setGearRatio`).
- **Tuning:** a good order is kG until it holds still, then kS, then kV, then kP until it tracks well, then a little kD if it overshoots.
- **What still applies:** soft limits, continuous mode and rotation limits all still work. `motorSpeed`, `motorUpSpeed`, `motorDownSpeed`, `minSpeed`, `holdSpeed`, `kP` and `accelerationTime` don't apply to Motion Magic moves, but `drive` still uses them.
- **Sensor:** Motion Magic uses the TalonFX's own sensor (built-in or `setCANcoder`), not a position supplier.

### Spinning at a velocity

`setVelocity` makes the TalonFX hold a speed even under load, e.g. a shooter that shouldn't slow down when a game piece goes through.

```java
Motor shooter = new Motor(14)
    .setFeedforward(0, 0.12, 0)  // kV: about 12 / top speed in rotations per second
    .setPID(0.1, 0, 0);

controller.rightTrigger().whileTrue(shooter.velocityCommand(80)); // 80 rotations/s while held
new Trigger(shooter::isAtVelocity).onTrue(feedCommand());          // feed once it's up to speed
```

| Method | What it does |
| --- | --- |
| `setVelocity(double velocity)` | Spins at `velocity` rotations per second (mechanism rotations with a gear ratio) until stopped or told to do something else. |
| `velocityCommand(double velocity)` | A command that spins at `velocity` while it runs and stops when it ends. |
| `isAtVelocity()` | True when there's a velocity target and the motor is within `velocityThreshold` (1 rotation/s by default) of it. |

Velocity uses the same `setPID` and `setFeedforward` gains as Motion Magic, so one motor can't have separate tuning for each. Mostly it's `kV` doing the work. Soft limits still apply.

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
Motor turret = new Motor(12).setGearRatio(10).setContinuous(0, 1);
turret.goTo(0.1); // from 0.9 this moves forward 0.2, not back 0.8
```

`setContinuous(min, max)` treats `min` and `max` as the same spot, so `goTo` always takes the shortest way around. This works no matter how many times the mechanism has spun. Use the length of one full turn in your position units. Here `setGearRatio(10)` makes the built-in encoder read in turret rotations, so one full turn is `0` to `1`. Without a gear ratio it reads motor rotations, and the same turret would need `setContinuous(0, 10)`.

#### Turrets with a rotation limit

Many turrets can only spin so far before their wires run out. Add soft limits to a continuous motor to set how far it can go each way:

```java
Motor turret = new Motor(12).setGearRatio(10).setContinuous(0, 1).setRange(-2, 3); // 2 turns back, 3 forward
```

`goTo(angle)` then treats the target as an angle and goes to whichever matching angle inside the limits is closest. If the short way would cross a limit, it goes the long way round instead. `drive` stops at the limits as usual. This needs a position that keeps counting past a full turn, like the built-in encoder, rather than one that wraps back to 0. Zero the turret at a known angle with `resetPosition(0)`, a limit switch or `homeCommand` (see below).

### Homing

A mechanism's encoder starts at 0 wherever the mechanism happens to be when the robot turns on. Homing finds the real zero.

```java
// Drive down slowly until the motor stalls against the bottom (20 A) or the reverse limit switch is pressed, then call that 0.
autoChooser.addOption("Home elevator", elevator.homeCommand(-0.15, 20, 0));
```

`homeCommand(speed, stallCurrent, position)` ignores the soft limits while it runs, since they're meaningless until the mechanism is zeroed. It waits a quarter of a second before watching the current, because starting up draws a short burst. If it's interrupted, it stops without resetting. A good `stallCurrent` is somewhere between what `getStatorCurrent()` shows while it's moving freely and what it shows while pushing against the stop.

With a limit switch, `setReverseLimitSwitchPosition(0.0)` re-zeroes every time the switch is pressed, so you may not need a homing command at all.

### TalonFX settings

When a `Motor` is created, it reads the settings already on the TalonFX. Those are the TalonFX defaults unless you changed them in Phoenix Tuner. The setters below change those settings on the TalonFX itself.

```java
Motor elevator = new Motor(11)
    .setGearRatio(12)                      // positions in output-shaft rotations
    .setStatorCurrentLimit(60.0)           // push less hard (TalonFX default 120 A)
    .setSupplyCurrentLimit(40.0)           // draw less from the battery (TalonFX default 70 A)
    .setReverseLimitSwitchPosition(0.0);   // re-zero whenever the bottom switch is pressed

elevator.resetPosition(0); // or zero it by hand, e.g. at startup
```

- **Gear ratio:** `setGearRatio(ratio)` is the number of motor rotations per mechanism rotation. The built-in encoder then reads in mechanism rotations, so soft limits, targets, velocities, `threshold` and all the gains use those units. If you add a gear ratio after tuning, retune.
- **CANcoder:** `setCANcoder(canId, rotorToSensorRatio)` makes the TalonFX read its position from a CANcoder on the same CAN bus, e.g. an absolute encoder on an arm so it knows where it is at startup. `rotorToSensorRatio` is motor rotations per CANcoder rotation. After this, `setGearRatio` means CANcoder rotations per mechanism rotation, which is 1 when the CANcoder is on the mechanism itself. Set the CANcoder's zero in Phoenix Tuner.
- **Current limits:** pass `null` to turn a limit off. Once the supply limit has been reached for a while, the TalonFX drops to a lower limit. `setSupplyCurrentLowerLimit(amps, seconds)` changes that, and the default is 40 A after 1 second.
- **Limit switches:** the TalonFX already stops the motor while a limit switch wired to it is pressed. The forward switch stops forward driving and the reverse switch stops reverse driving, inverted or not. `setLimitSwitchesEnabled(false)` turns that off. `setForwardLimitSwitchPosition` and `setReverseLimitSwitchPosition` also reset the encoder whenever that switch is pressed, so the mechanism re-zeroes itself.
- **Everything else:** `configure` gives you the full Phoenix 6 `TalonFXConfiguration` for any other TalonFX setting, and applies it:

```java
motor.configure(c -> {
    c.Audio.BeepOnBoot = false;
    c.MotionMagic.MotionMagicJerk = 40;                                  // smooth out Motion Magic further
    c.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;
});
```

`drive` and the default `goTo` use plain percent output. Motion Magic and `setVelocity` use the TalonFX's voltage control with gain slot 0. Settings for other control modes (`Slot1`, `Slot2`, `TorqueCurrent`) have no effect. Settings that duplicate `Motor` features (`OpenLoopRamps`, `SoftwareLimitSwitch`, `MotorOutput.PeakForwardDutyCycle`) do work, but they stack with `Motor`'s own version, so use one or the other.

### Readings

| Method | What it returns |
| --- | --- |
| `getPosition()` | Position (see above). |
| `getVelocity()` | Rotations per second from the TalonFX's sensor (mechanism rotations with a gear ratio). |
| `getStatorCurrent()` | Amps through the motor, which shows how hard it's pushing. Negative while pushing backwards. |
| `getSupplyCurrent()` | Amps drawn from the battery. |
| `getTemperature()` | The TalonFX's temperature in °C. It starts limiting itself around 100 °C. |
| `isForwardLimitSwitchPressed()` / `isReverseLimitSwitchPressed()` | Whether that limit switch is pressed. Normally-closed switches are accounted for, as long as you've set them to normally closed with `configure`. |

### Dashboard

```java
new Motor(11).setDashboardName("Elevator");
```

The motor then publishes these to NetworkTables under `Motors/Elevator` every loop:

- position, target and whether it's at target
- velocity and target velocity
- output, stator current, supply current and temperature
- whether it's enabled

Open them in Elastic, AdvantageScope or Shuffleboard. A target with nothing set shows as `NaN`. Pass `null` to stop publishing.

### Simulation

`Motor` works in the WPILib robot simulator with no changes. Each loop it feeds the TalonFX a simulated motor, so `drive`, `goTo`, Motion Magic, `setVelocity`, soft limits and the dashboard all behave close to the real thing. That's handy for checking commands and getting rough gains before the robot exists.

```java
new Motor(11).setGearRatio(12).setSimulation(DCMotor.getFalcon500(1), 0.05);
```

`setSimulation(gearbox, momentOfInertia)` sets what's modelled. The default is a Kraken X60 turning 0.01 kg·m², and a heavier mechanism needs a bigger value. The simulation is a free-spinning mechanism: it doesn't model gravity, hard stops or limit switches. Only the built-in encoder is simulated, not a CANcoder or position supplier. Expect to retune gains on the real robot.

### All settings

Every setter returns the motor, so you can chain them.

| Setter | Default | What it does |
| --- | --- | --- |
| `setInverted(boolean)` | `false` | Flips the direction. This is the TalonFX's own inversion setting, so the encoder and limit switches flip to match. |
| `setRange(min, max)` | no limits | Soft limits (see above). `setMinValue` and `setMaxValue` set one side. |
| `setContinuous(min, max)` | off | Shortest-way-around for turrets. Add soft limits for a rotation limit (see above). |
| `setGearRatio(double)` | `1` | Motor rotations per mechanism rotation, so the encoder reads in mechanism rotations. |
| `setStatorCurrentLimit(Double)` | TalonFX setting (120 A) | Caps current through the motor, which limits how hard it can push. `null` turns it off. |
| `setSupplyCurrentLimit(Double)` | TalonFX setting (70 A) | Caps current drawn from the battery. `null` turns it off. |
| `setSupplyCurrentLowerLimit(amps, seconds)` | TalonFX setting (40 A after 1 s) | The supply limit the TalonFX drops to once the supply limit has been hit for `seconds`. |
| `setLimitSwitchesEnabled(boolean)` | TalonFX setting (`true`) | Whether the TalonFX stops the motor at its limit switches. |
| `setForwardLimitSwitchPosition(Double)` | `null` (off) | Resets the encoder to this position whenever the forward limit switch is pressed. |
| `setReverseLimitSwitchPosition(Double)` | `null` (off) | Same, for the reverse limit switch. |
| `setCANcoder(canId, rotorToSensorRatio)` | built-in encoder | Read the position from a CANcoder (see above). |
| `setMotionMagic(maxVelocity, maxAcceleration)` | off | Runs `goTo` on the TalonFX (see above). |
| `setPID(kP, kI, kD)` | TalonFX setting (0) | TalonFX gains for Motion Magic and velocity, in volts. |
| `setFeedforward(kS, kV, kA)` | TalonFX setting (0) | TalonFX feedforward for Motion Magic and velocity, in volts. |
| `setGravity(kG, arm)` | TalonFX setting (0, elevator) | Volts to hold the mechanism up against gravity. |
| `setVelocityThreshold(double)` | `1.0` | How close to the velocity target counts as being there, in rotations per second. |
| `setDashboardName(String)` | `null` (off) | Publishes telemetry to NetworkTables (see above). |
| `setSimulation(DCMotor, double)` | Kraken X60, 0.01 kg·m² | What the simulator models (see above). |
| `configure(Consumer<TalonFXConfiguration>)` | TalonFX settings | Changes any other TalonFX setting (see above). |
| `setMotorSpeed(double)` | `1.0` | Speed for `drive()`, and the top speed for `goTo`. |
| `setMotorUpSpeed(Double)` | `motorSpeed` | Top `goTo` speed toward higher positions. `null` falls back to `motorSpeed`. |
| `setMotorDownSpeed(Double)` | `motorSpeed` | Top `goTo` speed toward lower positions. `null` falls back to `motorSpeed`. |
| `setMinSpeed(double)` | `0.0` | Smallest non-zero speed, so the motor doesn't stall on friction near a target. |
| `setAccelerationTime(double)` | `0.0` (off) | Seconds from stopped to full speed (see above). |
| `setKP(double)` | `0.1` | How hard `goTo` pushes: speed = kP × distance. Higher is faster but can overshoot. |
| `setThreshold(double)` | `0.025` | How close to the target counts as being there. |
| `setHoldSpeed(double)` | `0.0` | Output while holding at the target, e.g. a little upward power for an elevator. |
| `setPositionSupplier(DoubleSupplier)` | TalonFX encoder | Read the position from any other sensor in robot code. Its value is used as-is. For a CANcoder, `setCANcoder` is better because it also works with Motion Magic. |
| `setEnabled(boolean)` | `true` | Disabling stops the motor and ignores `drive`/`goTo` until it's enabled again. `toggleEnabled()` and `isEnabled()` also exist. |
| `setNeutralMode(NeutralModeValue)` | TalonFX setting (`Coast`) | `Brake` resists turning when there's no power; `Coast` spins freely. |

### Other methods

| Method | What it does |
| --- | --- |
| `resetPosition(double position)` | Tells the built-in encoder it's now at `position`, e.g. `resetPosition(0)` with an elevator at the bottom. It doesn't move the motor, and it has no effect on a position supplier. |
| `homeCommand(speed, stallCurrent, position)` | A command that finds the mechanism's zero (see above). |
| `brakelessReset(double seconds)` | A command that stops the motor and coasts for `seconds`, e.g. so a mechanism can settle against a hard stop, then switches back to brake mode. |
| `getMotor()` / `getTalonFX()` | The underlying TalonFX, for anything `Motor` doesn't cover. Don't call `set()` or `setControl()` on it yourself, because `Motor` overwrites the output every loop. Change settings with `configure` instead of the TalonFX's configurator, or `Motor` may undo them later. |
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

Each motor uses its own position. If one motor is mounted the other way, create it with `inverted = true` so both count up in the same direction. Alternatively, give the group one shared sensor with `setPositionSupplier` or `setCANcoder`. `group.setInverted(...)` sets every motor to the same value. `getMotors()` returns the motors in the group, e.g. for the readings.

A few methods work slightly differently on a group:

- **`homeCommand`:** stops every motor as soon as any one of them stalls or hits a limit switch, then resets them all.
- **`setGravity(kG, arm)`:** each motor adds the full `kG`, so split the voltage needed to hold the mechanism between the motors.
- **`setSimulation`:** each motor is simulated on its own, so give each one its share of the moment of inertia.
- **`setDashboardName("Elevator")`:** publishes the motors as `Motors/Elevator 1`, `Motors/Elevator 2` and so on.

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
