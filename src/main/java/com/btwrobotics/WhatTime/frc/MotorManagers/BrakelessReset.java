package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that temporarily disables brake mode on motors to allow natural settling.
 * 
 * <p>This command sets motors to coast mode for a specified duration, allowing them
 * to fall to their resting position naturally without resistance. After the timeout,
 * it executes reset actions (typically encoder resets) for each motor. This is useful
 * for recalibrating encoders when the mechanism reaches a known physical stop.
 * 
 * <p>The command finishes automatically after the specified duration elapses.
 * 
 * @see MotorResetPair
 * @see MotorWrapper
 */
public class BrakelessReset extends Command {
    /** The duration in seconds to keep motors in coast mode. */
    private final double duration;
    
    /** List of motor-action pairs to process during the reset. */
    private final List<MotorResetPair> pairs;
    
    /** Timer used to track the elapsed time during coast mode. */
    private final Timer timer = new Timer();

    /**
     * Pairs a motor with its reset action to be executed after coasting.
     * 
     * <p>This class bundles a {@link MotorWrapper} with a {@link Runnable} that
     * will be executed when the brakeless reset completes. The reset action
     * typically involves resetting the motor's encoder to a known position.
     */
    public static class MotorResetPair {
        /** The motor to be set to coast mode. */
        public final MotorWrapper motor;
        
        /** The action to execute after the coast period ends. */
        public final Runnable resetAction;
        
        /**
         * Creates a new motor-action pair.
         * 
         * @param motor the motor wrapper to control
         * @param resetAction the action to run after coasting completes
         */
        public MotorResetPair(MotorWrapper motor, Runnable resetAction) {
            this.motor = motor;
            this.resetAction = resetAction;
        }
    }

    /**
     * Creates a new BrakelessReset command.
     * 
     * @param duration the time in seconds to keep motors in coast mode
     * @param pairs list of motor-action pairs to process; must not be null or empty
     * @throws IllegalArgumentException if pairs is null or empty
     */
    public BrakelessReset(double duration, List<MotorResetPair> pairs) {
        this.duration = duration;
        this.pairs = pairs;
        if (pairs == null || pairs.isEmpty()) {
            throw new IllegalArgumentException("pairs cannot be null or empty");
        }
    }

    /**
     * Initializes the command by setting all motors to coast mode and starting the timer.
     * 
     * <p>Called when the command is first scheduled. Sets each motor's neutral mode
     * to coast, allowing them to move freely, then starts timing the coast period.
     */
    @Override
    public void initialize() {
        // Set all motors to coast
        for (MotorResetPair pair : pairs) {
            pair.motor.getMotor().setNeutralMode(NeutralModeValue.Coast);
        }
        // Reset time to 0 and then start
        timer.reset();
        timer.start();
    }

    /**
     * Determines whether the command has finished executing.
     * 
     * @return true if the coast duration has elapsed, false otherwise
     */
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    /**
     * Cleans up when the command ends by stopping the timer and executing reset actions.
     * 
     * <p>Called when the command ends, either normally or when interrupted.
     * Stops the timer and runs each motor's reset action (typically encoder reset).
     * This ensures motors are properly recalibrated after reaching their rest position.
     * 
     * @param interrupted true if the command was interrupted, false if it ended normally
     */
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        // Run each reset action
        for (MotorResetPair pair : pairs) {
            pair.resetAction.run();
        }
    }
}
