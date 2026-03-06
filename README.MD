# WhatTime (Python)

WhatTime is a RobotPy utility package for common FRC patterns:
- Motor abstraction for CTRE Phoenix 6 (`TalonFX`)
- Group control for multiple motors
- Driver station alliance helpers

This repository is currently Python-first and command-based (`commands2`).

## Use Cases

### 1. Single-motor position control with safety limits
Use `Motor` when you want one motor to:
- move to a target (`goto`)
- optionally run open-loop speed (`drive`, free mode only)
- enforce min/max bounds when not in free mode
- apply hold behavior and P-gain based correction in a default command

### 2. Synchronized control across multiple motors
Use `MotorGroup` when you want to run the same command across a list of `Motor` objects:
- enable/disable all motors
- broadcast `goto`/`drive`
- bulk neutral mode changes
- run a group brakeless reset command

### 3. Alliance-aware robot behavior
Use `AllianceManager` to:
- query current alliance
- get the opposing alliance
- toggle alliance values safely

## Installation

### For local development
```bash
pip install -e .
```

### In a robot project
Add this package to your robot project's dependencies (editable path or published package), then import from `whattime_frc`.

## Package Structure

```text
src/whattime_frc/
  DriverStation/
    AllianceManager.py
  MotorManagers/
    Motor.py
    MotorGroup.py
```

## Motor API

Import:
```python
from whattime_frc.MotorManagers import Motor
```

### Constructor
`Motor(*talonfx_args, ..., **talonfx_kwargs)` forwards positional/keyword args directly to `TalonFX`, plus WhatTime settings:
- `inverted: bool = False`
- `minValue: float = -1.0`
- `maxValue: float = 1.0`
- `minSpeed: float = 0.0`
- `motorSpeed: float = 1.0`
- `free: bool = False`
- `holdSpeed: float = 0.0`
- `threshold: float = 0.025`
- `pG: float = 1.0`
- `currentValueSupplier: Callable[[], float] | None = None`

### Behavior notes
- `Motor` is a `commands2.Subsystem`.
- A default command is installed automatically and calls `update()`.
- Motors start disabled (`_isEnabled = False`). You must call `enable()` or `toggleEnabled()`.
- If `free=False`:
  - `drive()` is disabled (raises `RuntimeError`)
  - `goto(target)` clamps to `[minValue, maxValue]`
- If `free=True`:
  - `drive()` is allowed
  - `goto(target)` wraps for fastest path behavior

### Example: bounded position motor
```python
from whattime_frc.MotorManagers import Motor

arm = Motor(
    12,                  # TalonFX device ID
    "canivore",         # optional CAN bus
    minValue=0.0,
    maxValue=1.0,
    motorSpeed=0.5,
    minSpeed=0.05,
    holdSpeed=0.03,
    threshold=1.0,
    pG=0.015,
    free=False,
)

arm.enable()
arm.goto(0.5) # 90 degrees
```

### Example: free/continuous motor
```python
wheel = Motor(
    8,
    free=True,
    motorSpeed=0.4,
)

wheel.enable()
wheel.drive()      # uses motorSpeed default (0.4)
# wheel.drive(0.25)
# wheel.goto(3.14)  # allowed in free mode; wraps for shortest path
```

## MotorGroup API

Import:
```python
from whattime_frc.MotorManagers import Motor, MotorGroup
```

### Constructor
```python
group = MotorGroup([motor_a, motor_b, motor_c])
```

### Group methods
- `enable()`, `disable()`, `toggleEnabled()`, `isEnabled()`
- `drive()` / `drive(speed)`
- `goto(target)`
- `setNeutralMode(neutralModeValue)`
- `brakelessReset(duration) -> Command`
- `getMotors()`

### Example: group control
```python
from whattime_frc.MotorManagers import Motor, MotorGroup

left = Motor(1, free=False, minValue=0, maxValue=1.0, pG=0.02)
right = Motor(2, free=False, minValue=0, maxValue=1.0, pG=0.02, inverted=True)

group = MotorGroup([left, right])
group.enable()
group.goto(0.75)
```

### Example: brakeless reset command
```python
# returns a commands2 Command: coast -> wait -> brake
cmd = group.brakelessReset(0.75)
cmd.schedule()
```

## DriverStation API

Import:
```python
from whattime_frc.DriverStation import AllianceManager
```

### Methods
- `AllianceManager.getCurrentAlliance() -> Alliance | None`
- `AllianceManager.getOpposingAlliance() -> Alliance | None`
- `AllianceManager.toggleAlliance(alliance) -> Alliance | None`

### Example
```python
from whattime_frc.DriverStation import AllianceManager

current = AllianceManager.getCurrentAlliance()
opposing = AllianceManager.getOpposingAlliance()
```

## Error Handling and Validation

The motor classes perform strict runtime validation and will raise:
- `TypeError` for invalid argument types
- `ValueError` for invalid numeric ranges/values
- `RuntimeError` for invalid mode usage (for example, calling `drive()` when `free=False`)

## Notes

- `Motor` depends on `phoenix6` and RobotPy `commands2`.
- `DriverStation.getAlliance()` may return `None` before field data is available.
