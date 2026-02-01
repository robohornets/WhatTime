package com.btwrobotics.WhatTime.frc.YearlyMethods.Rebuilt;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;
import com.btwrobotics.WhatTime.frc.DriverStation.AllianceManager;
import com.btwrobotics.WhatTime.frc.DriverStation.MatchTimeManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RebuiltHubManager {
    public MatchTimeManager matchTimeManager;

    public RebuiltHubManager(MatchTimeManager matchTimeManager) {
        this.matchTimeManager = matchTimeManager;
    }

    private Optional<Alliance> inactiveFirstAlliance;

    /* Times
     * 0 - Auto: Idk if it has a separate timer for auto
     * 1 - Transition Shift: 2:20-210 or 140s-130s
     * 2 - Phase 1: 2:10-1:45 or 130s-105s
     * 3 - Phase 2: 1:45-1:20 or 105s-80s
     * 4 - Phase 3: 1:20-0:55 or 80s-55s
     * 5 - Phase 4: 0:55-0:30 or 55s-30s
     * 6 - Endgame: 0:30-0:00 or 30s-0s
    */
    public List<Double> sectionStartTimes = new ArrayList<>();


    public Optional<Alliance> currentInactiveAlliance;

    /**
     * Updates the alliance that is inactive first after autonomous.
     */
    public void updateInitialInactiveAlliance() {
        // Gets R or B from DriverStation for which hub is inactive first. Runs at the end of auto.
        String allianceString = DriverStation.getGameSpecificMessage();

        if (allianceString.equals("R")) {
            inactiveFirstAlliance = Optional.of(Alliance.Red);
        }
        else if (allianceString.equals("B")) {
            inactiveFirstAlliance = Optional.of(Alliance.Blue);
        }
    }

    /**
     * Adds triggers for commands at each phase of the match.
     */
    public void scheduleAllInactiveHubChanges() {
        // Transition Shift: 140s-130s
        matchTimeManager.scheduleEventAtTime(
            140, 
            enableAllianceCommand(),
            "Transition Shift"
        );

        // Phase 1: 130s-105s
        matchTimeManager.scheduleEventAtTime(
            130, 
            startInactiveManager(),
            "Teleop Phase 1"
        );

        // Phase 2: 105s-80s
        matchTimeManager.scheduleEventAtTime(
            105, 
            toggleAllianceCommand(),
            "Teleop Phase 2"
        );

        // Phase 3: 80s-55s
        matchTimeManager.scheduleEventAtTime(
            80, 
            toggleAllianceCommand(),
            "Teleop Phase 3"
        );

        // Phase 4: 55s-30s
        matchTimeManager.scheduleEventAtTime(
            55, 
            toggleAllianceCommand(),
            "Teleop Phase 4"
        );

        // Endgame: 30s-0s
        matchTimeManager.scheduleEventAtTime(
            30, 
            enableAllianceCommand(),
            "Endgame"
        );
    }

    // Runs at the start of phase 1 to set the first inactive alliance
    private Command startInactiveManager() {
        return Commands.runOnce(
            () -> {
                currentInactiveAlliance = inactiveFirstAlliance;
            }
        );
    }

    // Switches between inactive alliances
    private Command toggleAllianceCommand() {
        return Commands.runOnce(
            () -> {
                AllianceManager.toggleAlliance(currentInactiveAlliance);
            }
        );
    }

    // Sets the current alliance to active
    private Command enableAllianceCommand() {
        return Commands.runOnce(
            () -> {
                currentInactiveAlliance = AllianceManager.getOpposingAlliance();
            }
        );
    }

    // Sets the current alliance to inactive
    private Command disableAllianceCommand() {
        return Commands.runOnce(
            () -> {
                currentInactiveAlliance = AllianceManager.getCurrentAlliance();
            }
        );
    }

    /**
     * Indicates whether the current alliance's hub is active
     * @return {@code true} when hub is active
     */
    public boolean hubIsActive() {
        return !currentInactiveAlliance.equals(AllianceManager.getCurrentAlliance());
    }

    /**
     * Tells which alliance is inactive first
     * @return an {@code Optional<Alliance>} object of .Red or .Blue
     */
    public Optional<Alliance> getInactiveFirstAlliance() {
        return inactiveFirstAlliance;
    }

    public void putPhasesToNetworkTables() {
        NetworkTablesUtil.put("Upcoming Phases", matchTimeManager.pendingTriggerDescriptions);
        NetworkTablesUtil.put("Past Phases", matchTimeManager.completedTriggerDescriptions);
    }

    public void putPhasesToNetworkTables(String tableName) {
        NetworkTablesUtil.put(tableName, "Upcoming Phases", matchTimeManager.pendingTriggerDescriptions);
        NetworkTablesUtil.put(tableName, "Past Phases", matchTimeManager.completedTriggerDescriptions);
    }
}
