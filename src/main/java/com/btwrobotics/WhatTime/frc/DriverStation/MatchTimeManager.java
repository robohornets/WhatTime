package com.btwrobotics.WhatTime.frc.DriverStation;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Runs commands at set times during teleop, e.g. rumbling the controller with 30 seconds left.
 */
public class MatchTimeManager extends SubsystemBase {
    /** Descriptions of the scheduled events that haven't run yet. */
    public List<String> pendingTriggerDescriptions = new ArrayList<>();
    /** Descriptions of the scheduled events that have already run. */
    public List<String> completedTriggerDescriptions = new ArrayList<>();

    /**
     *  Schedules a trigger to run with a certain time (in seconds) remaining in the match
     * @param timeRemaining the time remaining in the match in seconds
     * @param eventCommand a WPILib Command that will run at the time of the event
     * @param triggerName an optional description of the trigger for reference in a List
     */
    public void scheduleEventAtTime(double timeRemaining, Command eventCommand, String triggerName) {
        String triggerDescription = "Trigger: " + timeRemaining + "s - " + triggerName;
        pendingTriggerDescriptions.add(triggerDescription);

        // Teleop only: during auto, getMatchTime() counts down auto's 15s, which would fire every event at once.
        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= timeRemaining && DriverStation.getMatchTime() > 0)
            .onTrue(
                eventCommand
                .andThen(Commands.runOnce(
                    () -> {
                        pendingTriggerDescriptions.remove(triggerDescription);
                        completedTriggerDescriptions.add(triggerDescription);
                    }
                ))
            );
    }

    /**
     *  Schedules a trigger to run with a certain time (in seconds) remaining in the match
     * @param timeRemaining the time remaining in the match in seconds
     * @param eventCommand a WPILib Command that will run at the time of the event
     */
    public void scheduleEventAtTime(double timeRemaining, Command eventCommand) {
        scheduleEventAtTime(timeRemaining, eventCommand, "");
    }
}
