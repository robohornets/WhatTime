from __future__ import annotations

from typing import Any

from commands2 import Command
from commands2 import cmd as Commands
from wpilib import DriverStation

from ...DriverStation import AllianceManager

Alliance = DriverStation.Alliance


class RebuiltHubManager:
    def __init__(self, matchTimeManager: Any) -> None:
        self.matchTimeManager = matchTimeManager
        self.inactiveFirstAlliance: Alliance | None = None
        self.currentInactiveAlliance: Alliance | None = None

        # Times:
        # 0 - Auto
        # 1 - Transition Shift: 140s-130s
        # 2 - Phase 1: 130s-105s
        # 3 - Phase 2: 105s-80s
        # 4 - Phase 3: 80s-55s
        # 5 - Phase 4: 55s-30s
        # 6 - Endgame: 30s-0s
        self.sectionStartTimes: list[float] = []

    def updateInitialInactiveAlliance(self) -> None:
        allianceString = DriverStation.getGameSpecificMessage()

        if allianceString == "":
            return

        if allianceString == "R":
            self.inactiveFirstAlliance = Alliance.kRed
        elif allianceString == "B":
            self.inactiveFirstAlliance = Alliance.kBlue

    def scheduleAllInactiveHubChanges(self) -> None:
        self.matchTimeManager.scheduleEventAtTime(
            140,
            self.enableAllianceCommand(),
            "Transition Shift",
        )

        self.matchTimeManager.scheduleEventAtTime(
            130,
            self.startInactiveManager(),
            "Teleop Phase 1",
        )

        self.matchTimeManager.scheduleEventAtTime(
            105,
            self.toggleAllianceCommand(),
            "Teleop Phase 2",
        )

        self.matchTimeManager.scheduleEventAtTime(
            80,
            self.toggleAllianceCommand(),
            "Teleop Phase 3",
        )

        self.matchTimeManager.scheduleEventAtTime(
            55,
            self.toggleAllianceCommand(),
            "Teleop Phase 4",
        )

        self.matchTimeManager.scheduleEventAtTime(
            30,
            self.enableAllianceCommand(),
            "Endgame",
        )

    def startInactiveManager(self) -> Command:
        def _run() -> None:
            if self.inactiveFirstAlliance is None:
                self.updateInitialInactiveAlliance()

            self.currentInactiveAlliance = self.inactiveFirstAlliance

        return Commands.runOnce(_run)

    def toggleAllianceCommand(self) -> Command:
        def _run() -> None:
            if self.currentInactiveAlliance is not None:
                self.currentInactiveAlliance = AllianceManager.toggleAlliance(
                    self.currentInactiveAlliance
                )
            else:
                self.currentInactiveAlliance = AllianceManager.getOpposingAlliance()

        return Commands.runOnce(_run)

    def enableAllianceCommand(self) -> Command:
        def _run() -> None:
            opposingAlliance = AllianceManager.getOpposingAlliance()
            if opposingAlliance is not None:
                self.currentInactiveAlliance = opposingAlliance

        return Commands.runOnce(_run)

    def disableAllianceCommand(self) -> Command:
        def _run() -> None:
            currentAlliance = AllianceManager.getCurrentAlliance()
            if currentAlliance is not None:
                self.currentInactiveAlliance = currentAlliance

        return Commands.runOnce(_run)

    def hubIsActive(self) -> bool:
        if self.currentInactiveAlliance is None:
            return False

        currentAlliance = AllianceManager.getCurrentAlliance()
        if currentAlliance is None:
            return False

        return self.currentInactiveAlliance != currentAlliance

    def getInactiveFirstAlliance(self) -> Alliance | None:
        return self.inactiveFirstAlliance

    # def putPhasesToNetworkTables(self, tableName: str | None = None) -> None:
    #     if NetworkTablesUtil is None:
    #         raise RuntimeError(
    #             "NetworkTablesUtil is not available at "
    #             "whattime_frc.DashboardManagers.NetworkTablesUtil."
    #         )

    #     if tableName is None:
    #         NetworkTablesUtil.put(
    #             "Upcoming Phases",
    #             self.matchTimeManager.pendingTriggerDescriptions,
    #         )
    #         NetworkTablesUtil.put(
    #             "Past Phases",
    #             self.matchTimeManager.completedTriggerDescriptions,
    #         )
    #         return

    #     NetworkTablesUtil.put(
    #         tableName,
    #         "Upcoming Phases",
    #         self.matchTimeManager.pendingTriggerDescriptions,
    #     )
    #     NetworkTablesUtil.put(
    #         tableName,
    #         "Past Phases",
    #         self.matchTimeManager.completedTriggerDescriptions,
    #     )
