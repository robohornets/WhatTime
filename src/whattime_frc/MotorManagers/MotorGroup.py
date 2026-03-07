from __future__ import annotations

from typing import Callable, overload
from numbers import Real

from commands2 import Command, Subsystem
from commands2 import cmd as Commands
from phoenix6.signals import NeutralModeValue

from .Motor import Motor

_UNSET = object()


class MotorGroup(Subsystem):
    def __init__(self, motors: list[Motor]) -> None:
        super().__init__()

        if len(motors) == 0:
            raise ValueError("motors cannot be empty.")

        self._motors = motors

    def toggleEnabled(self, enabled: bool | None = None) -> bool:
        allEnabled = True
        for motor in self._motors:
            allEnabled = allEnabled and motor.toggleEnabled(enabled)
        return allEnabled

    def isEnabled(self) -> bool:
        for motor in self._motors:
            if not motor.isEnabled():
                return False
        return True

    def setInverted(self, inverted: bool) -> MotorGroup:
        for motor in self._motors:
            motor.setInverted(inverted)
        return self

    def setMinValue(self, minValue: float) -> MotorGroup:
        for motor in self._motors:
            motor.setMinValue(minValue)
        return self

    def setMaxValue(self, maxValue: float) -> MotorGroup:
        for motor in self._motors:
            motor.setMaxValue(maxValue)
        return self

    def setRange(self, minValue: float, maxValue: float) -> MotorGroup:
        for motor in self._motors:
            motor.setRange(minValue, maxValue)
        return self

    def setMinSpeed(self, minSpeed: float) -> MotorGroup:
        for motor in self._motors:
            motor.setMinSpeed(minSpeed)
        return self

    def setMotorSpeed(self, motorSpeed: float) -> MotorGroup:
        for motor in self._motors:
            motor.setMotorSpeed(motorSpeed)
        return self

    def setFree(self, free: bool) -> MotorGroup:
        for motor in self._motors:
            motor.setFree(free)
        return self

    def setHoldSpeed(self, holdSpeed: float) -> MotorGroup:
        for motor in self._motors:
            motor.setHoldSpeed(holdSpeed)
        return self

    def setThreshold(self, threshold: float) -> MotorGroup:
        for motor in self._motors:
            motor.setThreshold(threshold)
        return self

    def setPG(self, pG: float) -> MotorGroup:
        for motor in self._motors:
            motor.setPG(pG)
        return self

    def setPositionSupplier(self, positionSupplier: Callable[[], float] | None) -> MotorGroup:
        for motor in self._motors:
            motor.setPositionSupplier(positionSupplier)
        return self

    @overload
    def drive(self) -> None: ...

    @overload
    def drive(self, speed: int | float) -> None: ...

    def drive(self, speed: object = _UNSET) -> None:
        if speed is _UNSET:
            for motor in self._motors:
                motor.drive()
            return

        if not isinstance(speed, Real):
            raise TypeError("speed must be an int or float.")

        scalar_speed = float(speed)
        for motor in self._motors:
            motor.drive(scalar_speed)

    def goTo(self, target: int | float) -> None:
        if not isinstance(target, Real):
            raise TypeError("target must be an int or float.")

        scalar_target = float(target)
        for motor in self._motors:
            motor.goTo(scalar_target)

    def setTarget(self, target: int | float) -> None:
        self.goTo(target)

    def goto(self, target: int | float) -> None:
        self.goTo(target)

    def setNeutralMode(self, neutralModeValue: NeutralModeValue) -> None:
        for motor in self._motors:
            motor.setNeutralMode(neutralModeValue)

    def brakelessReset(self, duration: int | float) -> Command:
        if not isinstance(duration, Real):
            raise TypeError("duration must be an int or float.")

        duration = float(duration)
        if duration < 0.0:
            raise ValueError("duration must be >= 0.")

        return Commands.sequence(
            Commands.runOnce(
                lambda: self.setNeutralMode(NeutralModeValue.COAST),
                self,
            ),
            Commands.waitSeconds(duration),
            Commands.runOnce(
                lambda: self.setNeutralMode(NeutralModeValue.BRAKE),
                self,
            ),
        )

    def getMotors(self) -> list[Motor]:
        return self._motors
