from __future__ import annotations

from typing import overload
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

    def toggleEnabled(self) -> bool:
        states = [motor.toggleEnabled() for motor in self._motors]
        return all(states)

    def enable(self) -> None:
        for motor in self._motors:
            motor.enable()

    def disable(self) -> None:
        for motor in self._motors:
            motor.disable()

    def isEnabled(self) -> bool:
        return all(motor.isEnabled() for motor in self._motors)

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

    def goto(self, target: int | float) -> None:
        if not isinstance(target, Real):
            raise TypeError("target must be an int or float.")

        scalar_target = float(target)
        for motor in self._motors:
            motor.goto(scalar_target)

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
    def getMotors(self):
        return self._motors