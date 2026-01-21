package com.btwrobotics.WhatTime.frc.DriverStation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MatchTimeManager extends SubsystemBase {
    public List<String> pendingTriggerDescriptions = new ArrayList<>();
    public List<String> completedTriggerDescriptions = new ArrayList<>();

    /**
     *  Schedules a trigger to run with a certain time (in seconds) remaining in the match
     * @param timeRemaining the time remaining in the match in seconds
     * @param eventCommand a WPILib Command that will run at the time of the event
     * @param triggerName an optional description of the trigger for reference in a List
     */
    public void scheduleEventAtTime(double timeRemaining, Command eventCommand, String triggerName) {
        String triggerDescription = "Trigger: " + timeRemaining + "s - " + triggerName;

        new Trigger(() -> DriverStation.getMatchTime() <= timeRemaining && DriverStation.getMatchTime() > 0)
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
        String triggerDescription = "Trigger: " + timeRemaining + "s - ";

        new Trigger(() -> DriverStation.getMatchTime() <= timeRemaining && DriverStation.getMatchTime() > 0)
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
}
