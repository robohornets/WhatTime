from __future__ import annotations

from wpilib import DriverStation

Alliance = DriverStation.Alliance


class AllianceManager:
    @staticmethod
    def getCurrentAlliance() -> Alliance | None:
        """
        Gets the current alliance for the robot from DriverStation.
        Returns None when alliance data is unavailable.
        """
        return DriverStation.getAlliance()

    @staticmethod
    def getOpposingAlliance() -> Alliance | None:
        """
        Gets the opposing alliance for the robot from DriverStation.
        Returns None when alliance data is unavailable.
        """
        return AllianceManager.toggleAlliance(DriverStation.getAlliance())

    @staticmethod
    def toggleAlliance(alliance: Alliance | None) -> Alliance | None:
        """
        Toggle between Red and Blue alliances.

        For Alliance values:
        - kBlue -> kRed
        - kRed -> kBlue

        If alliance is None (no data), returns None.
        Any other unexpected value falls back to kBlue.
        """
        if alliance is None:
            return None

        if alliance == Alliance.kBlue:
            return Alliance.kRed
        if alliance == Alliance.kRed:
            return Alliance.kBlue

        return Alliance.kBlue
