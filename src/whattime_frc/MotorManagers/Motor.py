from phoenix6.hardware import TalonFX


class Motor:
    def __init__(self, *talonfx_args, inverted: bool = False, **talonfx_kwargs) -> None:
        self._motor = TalonFX(*talonfx_args, **talonfx_kwargs)
        self.inverted = inverted

    def set(self, speed: float) -> None:
        actualSpeed = -speed if self.inverted else speed
        self._motor.set(actualSpeed)

    def getTalonFX(self) -> TalonFX:
        return self._motor

    def isInverted(self) -> bool:
        return self.inverted

    def getPosition(self) -> float:
        return self._motor.get_position().value
