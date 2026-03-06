# WhatTime (Python)

WhatTime is a RobotPy utility package for common FRC patterns.

Current modules:
- CTRE Phoenix 6 `TalonFX` motor abstraction (`Motor`)
- Multi-motor orchestration (`MotorGroup`)
- Driver Station alliance helpers (`AllianceManager`)
- Yearly methods (`RebuiltHubManager`)

## Installation

```bash
pip install -e .
```

Project metadata:
- Package: `whattime-frc`
- Python: `>=3.11`

## Package Structure

```text
src/whattime_frc/
  DriverStation/
    AllianceManager.py
  MotorManagers/
    Motor.py
    MotorGroup.py
  YearlyMethods/
    Rebuilt/
      RebuiltHubManager.py
```

## MotorManagers

Import:
```python
from whattime_frc.MotorManagers import Motor, MotorGroup
```

### `Motor`

`Motor` is a `commands2.Subsystem` wrapper around `phoenix6.hardware.TalonFX` with:
- enable/disable state control
- bounded target movement (`goto`) in non-free mode
- open-loop speed control (`drive`) in free mode
- auto-updating default command (`update()` loop)
- optional hold behavior and proportional correction (`pG`)

Common methods:
- `toggleEnabled()`, `isEnabled()`
- `goto(target)`
- `drive()` / `drive(speed)` (free mode only)
- `setNeutralMode(neutralModeValue)`
- `brakelessReset(duration)`

### `MotorGroup`

`MotorGroup` applies one action across a list of `Motor` instances.

Common methods:
- `toggleEnabled()`, `isEnabled()`
- `goto(target)`
- `drive()` / `drive(speed)`
- `setNeutralMode(neutralModeValue)`
- `brakelessReset(duration)`
- `getMotors()`

## DriverStation

Import:
```python
from whattime_frc.DriverStation import AllianceManager
```

Methods:
- `AllianceManager.getCurrentAlliance() -> Alliance | None`
- `AllianceManager.getOpposingAlliance() -> Alliance | None`
- `AllianceManager.toggleAlliance(alliance) -> Alliance | None`

## YearlyMethods (Rebuilt)

Import:
```python
from whattime_frc.YearlyMethods import RebuiltHubManager
```

`RebuiltHubManager` manages inactive/active hub state by match phase using command scheduling.

Primary methods:
- `updateInitialInactiveAlliance()`
- `scheduleAllInactiveHubChanges()`
- `startInactiveManager() -> Command`
- `toggleAllianceCommand() -> Command`
- `enableAllianceCommand() -> Command`
- `disableAllianceCommand() -> Command`
- `hubIsActive() -> bool`
- `getInactiveFirstAlliance() -> Alliance | None`

Expected `matchTimeManager` interface used by `RebuiltHubManager`:
- `scheduleEventAtTime(time_seconds, command, description)`

Example:
```python
from whattime_frc.YearlyMethods import RebuiltHubManager

hub = RebuiltHubManager(matchTimeManager)
hub.updateInitialInactiveAlliance()
hub.scheduleAllInactiveHubChanges()
```

## Notes

- This package depends on RobotPy (`commands2`, `wpilib`) and CTRE Phoenix 6 (`phoenix6`).
- Driver Station data (alliance/game-specific message) may be unavailable early in startup and can return `None`/empty data.
