from __future__ import annotations

from typing import Callable, overload
import math
import warnings

from commands2 import Command, Subsystem
from commands2 import cmd as Commands
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue

_UNSET = object()


class Motor(Subsystem):
    DEFAULT_INVERTED = False
    DEFAULT_MIN_VALUE = -1.0
    DEFAULT_MAX_VALUE = 1.0
    DEFAULT_MIN_SPEED = 0.0
    DEFAULT_MOTOR_SPEED = 1.0
    DEFAULT_FREE = True
    DEFAULT_HOLD_SPEED = 0.0
    DEFAULT_THRESHOLD = 0.025
    DEFAULT_PG = 0.1
    DEFAULT_POSITION_DOUBLE_SUPPLIER: Callable[[], float] | None = None

    def __init__(
        self,
        device_id: int,
        canbus: str | bool | None = None,
        inverted: bool = DEFAULT_INVERTED,
        minValue: float = DEFAULT_MIN_VALUE,
        maxValue: float = DEFAULT_MAX_VALUE,
        minSpeed: float = DEFAULT_MIN_SPEED,
        motorSpeed: float = DEFAULT_MOTOR_SPEED,
        free: bool = DEFAULT_FREE,
        holdSpeed: float = DEFAULT_HOLD_SPEED,
        threshold: float = DEFAULT_THRESHOLD,
        pG: float = DEFAULT_PG,
        currentValueSupplier: Callable[[], float] | None = DEFAULT_POSITION_DOUBLE_SUPPLIER,
    ) -> None:
        super().__init__()

        if isinstance(canbus, bool):
            inverted = canbus
            canbus = None

        if canbus is None:
            self._motor = TalonFX(device_id)
        else:
            self._motor = TalonFX(device_id, canbus)
        self._inverted = bool(inverted)

        self._minValue = self.DEFAULT_MIN_VALUE
        self._maxValue = self.DEFAULT_MAX_VALUE
        self._minSpeed = self.DEFAULT_MIN_SPEED
        self._motorSpeed = self.DEFAULT_MOTOR_SPEED
        self._free = self.DEFAULT_FREE
        self._holdSpeed = self.DEFAULT_HOLD_SPEED
        self._threshold = self.DEFAULT_THRESHOLD
        self._pG = self.DEFAULT_PG
        self._positionSupplier = self.DEFAULT_POSITION_DOUBLE_SUPPLIER

        self._targetValue = 0.0
        self._hasTarget = False
        self._isHolding = False
        self._isEnabled = False

        self._deccelerateSteps = 50

        self.setRange(minValue, maxValue)
        self.setMinSpeed(minSpeed)
        self.setMotorSpeed(motorSpeed)
        self.setFree(free)
        self.setHoldSpeed(holdSpeed)
        self.setThreshold(threshold)
        self.setPG(pG)
        self.setPositionSupplier(currentValueSupplier)

        self.setDefaultCommand(Commands.run(self._defaultCommand, self))

    @classmethod
    def of(cls, motor: TalonFX, inverted: bool) -> Motor:
        instance = cls.__new__(cls) # type: ignore
        Subsystem.__init__(instance)

        instance._motor = motor
        instance._inverted = bool(inverted)

        instance._minValue = cls.DEFAULT_MIN_VALUE
        instance._maxValue = cls.DEFAULT_MAX_VALUE
        instance._minSpeed = cls.DEFAULT_MIN_SPEED
        instance._motorSpeed = cls.DEFAULT_MOTOR_SPEED
        instance._free = cls.DEFAULT_FREE
        instance._holdSpeed = cls.DEFAULT_HOLD_SPEED
        instance._threshold = cls.DEFAULT_THRESHOLD
        instance._pG = cls.DEFAULT_PG
        instance._positionSupplier = cls.DEFAULT_POSITION_DOUBLE_SUPPLIER # type: ignore

        instance._targetValue = 0.0
        instance._hasTarget = False
        instance._isHolding = False
        instance._isEnabled = False
        instance._deccelerateSteps = 50

        instance.setDefaultCommand(Commands.run(instance._defaultCommand, instance))
        return instance

    def setInverted(self, inverted: bool) -> Motor:
        self._inverted = bool(inverted)
        return self

    def setMinValue(self, minValue: float) -> Motor:
        min_value = float(minValue)
        self._validateRange(min_value, self._maxValue)
        self._minValue = min_value
        return self

    def setMaxValue(self, maxValue: float) -> Motor:
        max_value = float(maxValue)
        self._validateRange(self._minValue, max_value)
        self._maxValue = max_value
        return self

    def setRange(self, minValue: float, maxValue: float) -> Motor:
        min_value = float(minValue)
        max_value = float(maxValue)
        self._validateRange(min_value, max_value)
        self._minValue = min_value
        self._maxValue = max_value
        return self

    def setMinSpeed(self, minSpeed: float) -> Motor:
        min_speed = float(minSpeed)
        self._validateFinite(min_speed, "minSpeed")
        self._validateNonNegative(min_speed, "minSpeed")
        if min_speed > self._motorSpeed:
            raise ValueError("minSpeed cannot exceed motorSpeed.")
        self._minSpeed = min_speed
        return self

    def setMotorSpeed(self, motorSpeed: float) -> Motor:
        motor_speed = float(motorSpeed)
        self._validateFinite(motor_speed, "motorSpeed")
        self._validateNonNegative(motor_speed, "motorSpeed")
        if self._minSpeed > motor_speed:
            raise ValueError("minSpeed cannot exceed motorSpeed.")
        self._motorSpeed = motor_speed
        return self

    def setFree(self, free: bool) -> Motor:
        self._free = bool(free)
        return self

    def setHoldSpeed(self, holdSpeed: float) -> Motor:
        hold_speed = float(holdSpeed)
        self._validateFinite(hold_speed, "holdSpeed")
        self._holdSpeed = hold_speed
        return self

    def setThreshold(self, threshold: float) -> Motor:
        threshold_value = float(threshold)
        self._validateFinite(threshold_value, "threshold")
        self._validateNonNegative(threshold_value, "threshold")
        self._threshold = threshold_value
        return self

    def setPG(self, pG: float) -> Motor:
        pg = float(pG)
        self._validateFinite(pg, "pG")
        self._pG = pg
        return self

    def setPositionSupplier(self, positionSupplier: Callable[[], float] | None) -> Motor:
        if positionSupplier is not None and not callable(positionSupplier):
            raise TypeError("positionSupplier must be callable or None.")
        self._positionSupplier = positionSupplier
        return self

    def toggleEnabled(self, enabled: bool | None = None) -> bool:
        if enabled is None:
            self._isEnabled = not self._isEnabled
        else:
            self._isEnabled = bool(enabled)
        return self._isEnabled

    def isEnabled(self) -> bool:
        return self._isEnabled

    @overload
    def drive(self) -> None: ...

    @overload
    def drive(self, speed: int | float) -> None: ...

    def drive(self, speed: object = _UNSET) -> None:
        if not self._free:
            raise RuntimeError(".drive() is disabled; use .goTo() when not using free rotation.")
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
                UserWarning,
            )

        self._targetValue = speed
        self._hasTarget = True
        self._isHolding = False

    def goTo(self, target: float) -> None:
        if not self._isEnabled:
            warnings.warn("Motor is disabled.", RuntimeWarning)
            return

        target = float(target)
        self._targetValue = self._wrapValue(target) if self._free else self._clamp(target, self._minValue, self._maxValue)
        self._hasTarget = True
        self._isHolding = False

    def setTarget(self, target: float) -> None:
        self.goTo(target)

    def goto(self, target: float) -> None:
        self.goTo(target)

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

        if currentValue >= self._maxValue and speed > 0.0:
            speed = 0.0
        if currentValue <= self._minValue and speed < 0.0:
            speed = 0.0

        self.set(speed)
        return speed

    def _update(self) -> float:
        return self.update()

    def _defaultCommand(self) -> None:
        self.update()

    def _calculateSpeedWithAcceleration(self, currentValue: float) -> float:
        distanceDifference = self._positionError(currentValue, self._targetValue)
        speed = self._pG * distanceDifference

        speed = self._clamp(speed, -self._motorSpeed, self._motorSpeed)

        if speed != 0.0 and abs(speed) < self._minSpeed:
            speed = math.copysign(self._minSpeed, speed)

        return speed

    def _positionError(self, currentValue: float, targetValue: float) -> float:
        if not self._free:
            return targetValue - currentValue

        span = self._maxValue - self._minValue
        raw = targetValue - currentValue
        shifted = raw + span / 2.0
        wrappedShifted = shifted - span * math.floor(shifted / span)
        return wrappedShifted - span / 2.0

    def _wrapValue(self, value: float) -> float:
        span = self._maxValue - self._minValue
        shifted = value - self._minValue
        return shifted - span * math.floor(shifted / span) + self._minValue

    def set(self, speed: float) -> None:
        speed = float(speed)
        actualSpeed = -speed if self._inverted else speed
        self._motor.set(actualSpeed)

    def getCurrentValue(self) -> float:
        if self._positionSupplier is not None:
            return float(self._positionSupplier())
        return self.getPosition()

    def getTalonFX(self) -> TalonFX:
        return self._motor

    def getMotor(self) -> TalonFX:
        return self._motor

    def isInverted(self) -> bool:
        return self._inverted

    def getPosition(self) -> float:
        return self._motor.get_position().value

    @staticmethod
    def _validateRange(minValue: float, maxValue: float) -> None:
        Motor._validateFinite(minValue, "minValue")
        Motor._validateFinite(maxValue, "maxValue")
        if maxValue <= minValue:
            raise ValueError("maxValue must be greater than minValue.")

    @staticmethod
    def _validateNonNegative(value: float, name: str) -> None:
        if value < 0.0:
            raise ValueError(f"{name} must be >= 0.")

    @staticmethod
    def _validateFinite(value: float, name: str) -> None:
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite.")

    @staticmethod
    def _clamp(value: float, minValue: float, maxValue: float) -> float:
        return max(minValue, min(maxValue, value))
