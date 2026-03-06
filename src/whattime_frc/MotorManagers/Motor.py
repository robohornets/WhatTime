from __future__ import annotations

from typing import Callable, overload
import math
import warnings

from commands2 import Command, Subsystem
from commands2 import cmd as Commands
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from phoenix6 import CANBus

_UNSET = object()


class Motor(Subsystem):
    def __init__(
        self,
        device_id: int,
        canbus: CANBus | str = CANBus(),
        inverted: bool = False,
        minValue: float = -1.0,
        maxValue: float = 1.0,
        minSpeed: float = 0.0,
        motorSpeed: float = 1.0,
        free: bool = False,
        holdSpeed: float = 0.0,
        threshold: float = 0.025,
        pG: float = 1.0,
        currentValueSupplier: Callable[[], float] | None = None
    ) -> None:
        super().__init__()

        if not math.isfinite(float(minValue)) or not math.isfinite(float(maxValue)):
            raise ValueError("minValue and maxValue must be finite.")
        if float(maxValue) <= float(minValue):
            raise ValueError("maxValue must be greater than minValue.")


        if not math.isfinite(float(motorSpeed)) or not math.isfinite(float(minSpeed)) or not math.isfinite(float(holdSpeed)):
            raise ValueError("motorSpeed, minSpeed, and holdSpeed must be finite.")
        if float(motorSpeed) < 0.0:
            raise ValueError("motorSpeed must be >= 0.")
        if float(minSpeed) < 0.0:
            raise ValueError("minSpeed must be >= 0.")
        if float(minSpeed) > float(motorSpeed):
            raise ValueError("minSpeed cannot exceed motorSpeed.")

        if not math.isfinite(float(threshold)):
            raise ValueError("threshold must be finite.")
        if float(threshold) < 0.0:
            raise ValueError("threshold must be >= 0.")

        if not math.isfinite(float(pG)):
            raise ValueError("pG must be finite.")

        if currentValueSupplier is not None and not callable(currentValueSupplier):
            raise TypeError("currentValueSupplier must be callable or None.")

        self._motor = TalonFX(device_id, canbus)
        self._inverted = inverted

        self._minValue = float(minValue)
        self._maxValue = float(maxValue)
        self._minSpeed = float(minSpeed)
        self._motorSpeed = float(motorSpeed)
        self._free = free
        self._holdSpeed = float(holdSpeed)
        self._threshold = float(threshold)
        self._pG = float(pG)
        self._currentValueSupplier = currentValueSupplier

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

    def drive(self, speed: object = _UNSET) -> None:
        if not self._free:
            raise RuntimeError(".drive() is disabled; use .goto() when not using free rotation.")
        if not self._isEnabled:
            warnings.warn("Motor is disabled.", RuntimeWarning)
            return

        if speed is _UNSET:
            speed = self._motorSpeed

        if not isinstance(speed, (int, float)):
            raise TypeError("speed must be an int or float.")

        speed = float(speed)
        if speed != 0.0 and abs(speed) < self._minSpeed:
            warnings.warn(
                f"Speed too slow ({abs(speed)} < {self._minSpeed}), running at minimum speed.",
                RuntimeWarning,
            )

        self._targetValue = speed
        self._hasTarget = True
        self._isHolding = False

    def goto(self, target: float) -> None:
        if not self._isEnabled:
            warnings.warn("Motor is disabled.", RuntimeWarning)
            return

        target = float(target)
        # In free mode, wrap the target so movement takes the fastest circular path.
        # In bounded mode, clamp target within configured limits.
        self._targetValue = self._wrapValue(target) if self._free else max(self._minValue, min(self._maxValue, target))
        self._hasTarget = True
        self._isHolding = False

    def setNeutralMode(self, neutralModeValue: NeutralModeValue) -> None:
        self._motor.setNeutralMode(neutralModeValue)

    def brakelessReset(self, duration: int | float) -> Command:
        duration = float(duration)
        if duration < 0.0:
            raise ValueError("duration must be >= 0.")

        return Commands.sequence(
            Commands.runOnce(lambda: self.setNeutralMode(NeutralModeValue.COAST), self),
            Commands.waitSeconds(duration),
            Commands.runOnce(lambda: self.setNeutralMode(NeutralModeValue.BRAKE), self),
        )

    def update(self) -> float:
        speed = 0.0

        if not self._isEnabled or not self._hasTarget:
            self.set(0.0)
            return 0.0

        if self._free:
            speed = max(-1.0, min(1.0, self._targetValue))
            self.set(speed)
            return speed

        currentValue = self.getCurrentValue()
        error = abs(self._positionError(currentValue, self._targetValue))

        if error <= self._threshold:
            self._isHolding = True
        elif error > self._threshold * 2:
            self._isHolding = False

        if self._isHolding:
            speed = self._holdSpeed
        else:
            speed = self._calculateSpeedWithAcceleration(currentValue)

        if not self._free:
            if currentValue >= self._maxValue and speed > 0.0:
                speed = 0.0
            if currentValue <= self._minValue and speed < 0.0:
                speed = 0.0

        self.set(speed)
        return speed

    def _defaultCommand(self) -> None:
        self.update()

    def _calculateSpeedWithAcceleration(self, currentValue: float) -> float:
        distanceDifference = self._positionError(currentValue, self._targetValue)
        speed = self._pG * distanceDifference

        speed = max(-self._motorSpeed, min(self._motorSpeed, speed))

        if speed != 0.0 and abs(speed) < self._minSpeed:
            speed = math.copysign(self._minSpeed, speed)

        return speed

    def _positionError(self, currentValue: float, targetValue: float) -> float:
        if not self._free:
            return targetValue - currentValue

        span = self._maxValue - self._minValue
        raw = targetValue - currentValue
        return ((raw + span / 2.0) % span) - span / 2.0

    def _wrapValue(self, value: float) -> float:
        span = self._maxValue - self._minValue
        return ((value - self._minValue) % span) + self._minValue

    def set(self, speed: float) -> None:
        speed = float(speed)
        actualSpeed = -speed if self._inverted else speed
        self._motor.set(actualSpeed)

    def getCurrentValue(self) -> float:
        if self._currentValueSupplier is not None:
            return float(self._currentValueSupplier())
        return self.getPosition()

    def getTalonFX(self) -> TalonFX:
        return self._motor

    def isInverted(self) -> bool:
        return self._inverted

    def getPosition(self) -> float:
        return self._motor.get_position().value
    
