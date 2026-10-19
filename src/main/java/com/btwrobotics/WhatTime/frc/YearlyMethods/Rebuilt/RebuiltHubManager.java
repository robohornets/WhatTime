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

/**
 * Tracks whether your alliance's hub is active during a 2026 REBUILT match. Both hubs are active
 * during auto, the transition shift and endgame; in between, the hubs take turns, starting with
 * the alliance named in the game-specific message.
 */
public class RebuiltHubManager {
    /** The MatchTimeManager used to schedule each phase change. */
    public MatchTimeManager matchTimeManager;

    /**
     * Creates a hub manager. Call {@link #scheduleAllInactiveHubChanges()} once afterwards to
     * start tracking.
     *
     * @param matchTimeManager the MatchTimeManager to schedule the phase changes on
     */
    public RebuiltHubManager(MatchTimeManager matchTimeManager) {
        this.matchTimeManager = matchTimeManager;
    }

    private Optional<Alliance> inactiveFirstAlliance = Optional.empty();

    /**
     * A place to keep phase start times. Not filled in by this class. The phases, by time left
     * in teleop, are:
     * <ul>
     *   <li>Transition Shift: 2:20-2:10 (140s-130s)
     *   <li>Phase 1: 2:10-1:45 (130s-105s)
     *   <li>Phase 2: 1:45-1:20 (105s-80s)
     *   <li>Phase 3: 1:20-0:55 (80s-55s)
     *   <li>Phase 4: 0:55-0:30 (55s-30s)
     *   <li>Endgame: 0:30-0:00 (30s-0s)
     * </ul>
     */
    public List<Double> sectionStartTimes = new ArrayList<>();

    /** The alliance whose hub is currently inactive, or empty when unknown. */
    public Optional<Alliance> currentInactiveAlliance = Optional.empty();

    /**
     * Updates the alliance that is inactive first after autonomous.
     */
    public void updateInitialInactiveAlliance() {
        // Gets R or B from DriverStation for which hub is inactive first
        String allianceString = DriverStation.getGameSpecificMessage();

        // Data not available yet
        if (allianceString == null || allianceString.isEmpty()) {
            return;
        }

        if (allianceString.equals("R")) {
            inactiveFirstAlliance = Optional.of(Alliance.Red);
        } else if (allianceString.equals("B")) {
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
                "Transition Shift");

        // Phase 1: 130s-105s
        matchTimeManager.scheduleEventAtTime(
                130,
                startInactiveManager(),
                "Teleop Phase 1");

        // Phase 2: 105s-80s
        matchTimeManager.scheduleEventAtTime(
                105,
                toggleAllianceCommand(),
                "Teleop Phase 2");

        // Phase 3: 80s-55s
        matchTimeManager.scheduleEventAtTime(
                80,
                toggleAllianceCommand(),
                "Teleop Phase 3");

        // Phase 4: 55s-30s
        matchTimeManager.scheduleEventAtTime(
                55,
                toggleAllianceCommand(),
                "Teleop Phase 4");

        // Endgame: 30s-0s
        matchTimeManager.scheduleEventAtTime(
                30,
                enableAllianceCommand(),
                "Endgame");
    }

    // Runs at the start of phase 1 to set the first inactive alliance
    private Command startInactiveManager() {
        return Commands.runOnce(
                () -> {
                    // Try to get data again if it is missing
                    if (!inactiveFirstAlliance.isPresent()) {
                        updateInitialInactiveAlliance();
                    }

                    currentInactiveAlliance = inactiveFirstAlliance;
                });
    }

    // Switches between inactive alliances
    private Command toggleAllianceCommand() {
        return Commands.runOnce(
                () -> {
                    if (currentInactiveAlliance.isPresent()) {
                        currentInactiveAlliance = AllianceManager.toggleAlliance(currentInactiveAlliance);
                    } else {
                        // Fallback: try to initialize from current alliance
                        currentInactiveAlliance = AllianceManager.getOpposingAlliance();
                    }
                });
    }

    // Sets the current alliance to active
    private Command enableAllianceCommand() {
        return Commands.runOnce(
                () -> {
                    Optional<Alliance> opposingAlliance = AllianceManager.getOpposingAlliance();
                    if (opposingAlliance.isPresent()) {
                        currentInactiveAlliance = opposingAlliance;
                    }
                });
    }

    /**
     * Indicates whether the current alliance's hub is active
     *
     * @return {@code true} when hub is active
     */
    public boolean hubIsActive() {
        // Both hubs are active during auto
        if (DriverStation.isAutonomous()) {
            return true;
        }

        // Return false (inactive) if no current inactive alliance data
        if (!currentInactiveAlliance.isPresent()) {
            return false;
        }

        Optional<Alliance> currentAlliance = AllianceManager.getCurrentAlliance();
        if (!currentAlliance.isPresent()) {
            return false;
        }

        return !currentInactiveAlliance.get().equals(currentAlliance.get());
    }

    /**
     * Tells which alliance is inactive first
     * 
     * @return an {@code Optional<Alliance>} object of .Red or .Blue
     */
    public Optional<Alliance> getInactiveFirstAlliance() {
        return inactiveFirstAlliance;
    }

    /**
     * Publishes the upcoming and past match phases to the default NetworkTables table
     * ("CustomDashboard"), as "Upcoming Phases" and "Past Phases".
     */
    public void putPhasesToNetworkTables() {
        NetworkTablesUtil.put("Upcoming Phases", matchTimeManager.pendingTriggerDescriptions);
        NetworkTablesUtil.put("Past Phases", matchTimeManager.completedTriggerDescriptions);
    }

    /**
     * Publishes the upcoming and past match phases to a NetworkTables table, as
     * "Upcoming Phases" and "Past Phases".
     *
     * @param tableName the name of the table to publish to
     */
    public void putPhasesToNetworkTables(String tableName) {
        NetworkTablesUtil.put(tableName, "Upcoming Phases", matchTimeManager.pendingTriggerDescriptions);
        NetworkTablesUtil.put(tableName, "Past Phases", matchTimeManager.completedTriggerDescriptions);
    }
}
