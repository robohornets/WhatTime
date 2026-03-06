from .Motor import Motor
from commands2 import Command, Subsystem
from commands2 import cmd as Commands
from phoenix6.signals import NeutralModeValue

from typing import overload, Callable
from numbers import Real
import math
import warnings

_UNSET = object()

class MotorGroup(Subsystem):
    def __init__(self,
        motors:         list[Motor],        # The list of motors that will be controlled (invertion will be respected)
        minValue:       float = -1.0,       # The minimum value the group is allowed to move to
        maxValue:       float = 1.0,        # The maximum value the group is allowed to move to
        minSpeed:       float = 0.0,        # Minimum speed of the motor to prevent stalling
        motorSpeed:     float = 1.0,        # Default speed the motor will use to move
        free:           bool = False,       # Allows the motors to spin freely without worrying about min or max
        looping:        bool = False,       # Wrap target/error to move along shortest path in the [minValue, maxValue) range
        holdSpeed:      float = 0.0,        # The speed of the motor to maintain its position
        threshold:      float = 0.025,      # The tolerance for determining is the target is reached
        pG:             float = 1.0,        # Proportional gain
        currentValueSupplier:   Callable[[], float] | None = None,# Function that will provide the position of the group (defaults to main motors position)
        mainMotor:              Motor | None = None,              # The primary motor used to measure the group's position
    ):
        super().__init__()

        if not isinstance(motors, list):
            raise TypeError("motors must be a list[Motor].")
        if len(motors) == 0:
            raise ValueError("motors cannot be empty.")
        if not all(isinstance(m, Motor) for m in motors):
            raise TypeError("all entries in motors must be Motor instances.")

        if not isinstance(free, bool):
            raise TypeError("free must be a bool.")
        if not isinstance(looping, bool):
            raise TypeError("looping must be a bool.")

        if not isinstance(minValue, Real) or isinstance(minValue, bool):
            raise TypeError("minValue must be a real number.")
        if not isinstance(maxValue, Real) or isinstance(maxValue, bool):
            raise TypeError("maxValue must be a real number.")
        if not math.isfinite(float(minValue)) or not math.isfinite(float(maxValue)):
            raise ValueError("minValue and maxValue must be finite.")
        if float(maxValue) <= float(minValue):
            raise ValueError("maxValue must be greater than minValue.")

        if not isinstance(motorSpeed, Real) or isinstance(motorSpeed, bool):
            raise TypeError("motorSpeed must be a real number.")
        if not isinstance(minSpeed, Real) or isinstance(minSpeed, bool):
            raise TypeError("minSpeed must be a real number.")
        if not isinstance(holdSpeed, Real) or isinstance(holdSpeed, bool):
            raise TypeError("holdSpeed must be a real number.")
        if not math.isfinite(float(motorSpeed)) or not math.isfinite(float(minSpeed)) or not math.isfinite(float(holdSpeed)):
            raise ValueError("motorSpeed, minSpeed, and holdSpeed must be finite.")
        if float(motorSpeed) < 0.0:
            raise ValueError("motorSpeed must be >= 0.")
        if float(minSpeed) < 0.0:
            raise ValueError("minSpeed must be >= 0.")
        if float(minSpeed) > float(motorSpeed):
            raise ValueError("minSpeed cannot exceed motorSpeed.")

        if not isinstance(threshold, Real) or isinstance(threshold, bool):
            raise TypeError("threshold must be a real number.")
        if not math.isfinite(float(threshold)):
            raise ValueError("threshold must be finite.")
        if float(threshold) < 0.0:
            raise ValueError("threshold must be >= 0.")

        if not isinstance(pG, Real) or isinstance(pG, bool):
            raise TypeError("pG must be a real number.")
        if not math.isfinite(float(pG)):
            raise ValueError("pG must be finite.")

        if currentValueSupplier is not None and not callable(currentValueSupplier):
            raise TypeError("currentValueSupplier must be callable or None.")
        if mainMotor is not None and not isinstance(mainMotor, Motor):
            raise TypeError("mainMotor must be a Motor or None.")
        if not free and currentValueSupplier is None and mainMotor is None:
            raise ValueError("In non-free mode, define currentValueSupplier or mainMotor.")
        if mainMotor is not None and mainMotor not in motors:
            warnings.warn("mainMotor is not in motors list; ensure this is intentional.", RuntimeWarning)

        self._motors = motors
        self._minValue = minValue
        self._maxValue = maxValue
        self._minSpeed = minSpeed
        self._motorSpeed = motorSpeed
        self._free = free
        self._looping = looping
        self._holdSpeed = holdSpeed
        self._threshold = threshold
        self._pG = pG
        self._currentValueSupplier = currentValueSupplier
        self._mainMotor = mainMotor

        self._targetValue = 0.0
        self._hasTarget = False
        self._isHolding = False
        self._isEnabled = False

        self.setDefaultCommand(Commands.run(self._defaultCommand, self))
        

    def toggleEnabled(self) -> bool:
        self._isEnabled = not self._isEnabled
        return self._isEnabled
    def enable(self) -> None:
        self._isEnabled = True
    def disable(self) -> None:
        self._isEnabled = False
    def isEnabled(self) -> bool:
        return self._isEnabled

    @overload
    def drive(self) -> None: ...
    @overload
    def drive(self, speed: int | float) -> None: ...
    def drive(self, speed: object = _UNSET):
        if not self._free: raise RuntimeError(".drive() is disabled; use .goto() when not using free rotation.")
        if not self._isEnabled: raise RuntimeError("MotorGroup is disabled.")

        if speed is _UNSET:
            speed = self._motorSpeed
        
        if not isinstance(speed, (int, float)): raise TypeError("Speed must be an int or float.")
        if speed != 0 and abs(speed) < self._minSpeed: warnings.warn(f"Speed too slow ({abs(speed)} < {self._minSpeed}), running at minimum speed.", RuntimeWarning)

        self._targetValue = speed
        self._hasTarget = True
        self._isHolding = False

    def goto(self, target: float) -> None:
        if self._free: raise RuntimeError(".goto() is disabled; use .drive() when using free rotation.")
        if not self._isEnabled: raise RuntimeError("MotorGroup is disabled.")
        if not isinstance(target, (int, float)): raise TypeError("target must be an int or float.")
        target = float(target)
        if not self._looping:
            if target > self._maxValue: raise ValueError(f"Target too large ({target} > {self._maxValue})")
            if target < self._minValue: raise ValueError(f"Target too small ({target} < {self._minValue})")

        self._targetValue = self._wrapValue(target) if self._looping else max(self._minValue, min(self._maxValue, target))
        self._hasTarget = True
        self._isHolding = False

    def setNeutralModeBulk(self, neutralModeValue: NeutralModeValue):
        # Set the neutral mode for all of the motors
        if not isinstance(neutralModeValue, NeutralModeValue): raise TypeError("neutralNodeValue must be of type NeutralModeValue")
        for motor in self._motors:
            motor.getTalonFX().setNeutralMode(neutralModeValue)

    def brakelessReset(self, duration: int | float) -> Command:
        # Temporarily disable brakes so the mechanism can settle naturally.
        if not isinstance(duration, (int, float)):
            raise TypeError("duration must be an int or float.")
        duration = float(duration)
        if duration < 0.0:
            raise ValueError("duration must be >= 0.")

        return Commands.sequence(
            Commands.runOnce(
                lambda: self.setNeutralModeBulk(NeutralModeValue.COAST),
                self,
            ),
            Commands.waitSeconds(duration),
            Commands.runOnce(
                lambda: self.setNeutralModeBulk(NeutralModeValue.BRAKE),
                self,
            ),
        )

    def _defaultCommand(self):
        if not self._isEnabled:
            self._setAllMotors(0.0)
            return
        
        if not self._hasTarget:
            self._setAllMotors(0.0)
            return
        
        speed = 0.0
        if self._free:
            # Free mode
            speed = max(-1.0, min(1.0, self._targetValue))
        else:
            # Target mode
            currentValue = self._getCurrentValue()
            error = abs(self._positionError(currentValue, self._targetValue))

            if error <= self._threshold:
                self._isHolding = True
            elif error > self._threshold * 2:
                self._isHolding = False

            if self._isHolding:
                speed = self._holdSpeed
            else:
                speed = self._calculateSpeedWithAcceleration()
                
            if not self._looping:
                if currentValue >= self._maxValue and speed > 0.0: speed = 0
                if currentValue <= self._minValue and speed < 0.0: speed = 0

        self._setAllMotors(speed)



    def _calculateSpeedWithAcceleration(self) -> float:
        if self._free: raise RuntimeError("function should not be used in free mode.")

        distanceDifference = self._positionError(self._getCurrentValue(), self._targetValue)
        speed = self._pG * distanceDifference

        speed = max(-self._motorSpeed, min(self._motorSpeed, speed))

        if speed != 0.0 and abs(speed) < self._minSpeed:
            speed = math.copysign(self._minSpeed, speed)

        return speed

    def _setAllMotors(self, speed: int | float) -> None:
        for motor in self._motors:
            motor.set(speed)

    def _getCurrentValue(self) -> float:
        if self._currentValueSupplier is not None:
            return float(self._currentValueSupplier())
        
        if self._mainMotor is not None:
            return self._mainMotor.getPosition()
        
        raise ValueError("Both mainMotor and currentValueSupplier are unset, please define one.")

    def _positionError(self, currentValue: float, targetValue: float) -> float:
        if not self._looping:
            return targetValue - currentValue

        span = self._maxValue - self._minValue
        raw = targetValue - currentValue
        return ((raw + span / 2.0) % span) - span / 2.0

    def _wrapValue(self, value: float) -> float:
        span = self._maxValue - self._minValue
        return ((value - self._minValue) % span) + self._minValue
