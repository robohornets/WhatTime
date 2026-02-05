package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Manager for controlling motor-driven mechanisms to specific positions with limits.
 * 
 * <p>This class manages position-based control of motor groups, ensuring they stay
 * within defined bounds while moving to target positions. It supports:
 * <ul>
 *   <li>Automatic speed control based on target position
 *   <li>Upper and lower limit enforcement
 *   <li>Position threshold for determining arrival at target
 *   <li>Hold speed for maintaining position when stopped
 * </ul>
 * 
 * <p>The manager uses a supplier pattern to obtain current position values,
 * allowing flexibility in how position is measured (encoders, sensors, etc.).
 * 
 * @see MotorWrapper
 */
public class PositionManager {
    /** The minimum allowed position value (lower limit). */
    private final double minValue;
    
    /** The maximum allowed position value (upper limit). */
    private final double maxValue;
    
    /** The list of motors controlled by this manager. */
    private final List<MotorWrapper> motors;
    
    /** The speed at which motors move toward target positions. */
    private final double motorSpeed;
    
    /** The speed applied to motors when holding position. */
    private final double holdSpeed;
    
    /** The tolerance for determining if target position is reached. */
    private final double threshold;
    
    /** Supplier that provides the current position value. */
    private final Supplier<Double> currentValueSupplier;
    
    /** The current target position for the motors to reach. */
    private double targetValue;
    
    /**
     * Creates a new PositionManager.
     * 
     * @param minValue the minimum allowed position (lower limit)
     * @param maxValue the maximum allowed position (upper limit)
     * @param motors the list of motors to control together
     * @param motorSpeed the speed for moving toward target (positive value)
     * @param holdSpeed the speed for holding position when stopped
     * @param threshold the tolerance for considering target reached
     * @param currentValueSupplier supplier that provides current position readings
     */
    public PositionManager(
        double minValue,
        double maxValue,
        List<MotorWrapper> motors,
        double motorSpeed,
        double holdSpeed,
        double threshold,
        Supplier<Double> currentValueSupplier
    ) {
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.motors = motors;
        this.targetValue = minValue; // target value is initially the minimum value
        this.motorSpeed = motorSpeed;
        this.holdSpeed = holdSpeed;
        this.threshold = threshold;
        this.currentValueSupplier = currentValueSupplier;
    }
    
    /** Internal flag tracking whether the current movement has completed. */
    private boolean isFinishedToggle = false;

    /**
     * Checks if the current movement operation has finished.
     * 
     * @return true if the target position has been reached or movement is blocked, false otherwise
     */
    public boolean isFinished() {
        return isFinishedToggle;
    }

    /**
     * Creates a command to move motors to the specified target position.
     * 
     * <p>This method returns a command that continuously updates motor speeds
     * to drive toward the target. The command will automatically:
     * <ul>
     *   <li>Respect upper and lower position limits
     *   <li>Stop when within threshold of target
     *   <li>Prevent movement beyond bounds
     * </ul>
     * 
     * <p>The command runs indefinitely and should be ended based on the
     * {@link #isFinished()} method or external conditions.
     * 
     * @param target the desired position to move to
     * @return a command that moves motors toward the target
     */
    public Command move(double target) {
        this.targetValue = target;
        this.isFinishedToggle = false;

        return Commands.run(() -> {
            updateMotorSpeed();
        });
    }


    
    /**
     * Gets the current target position.
     * 
     * @return the target position value
     */
    public double getTarget() {
        return targetValue;
    }


    /**
     * Stops motor movement and applies hold speed.
     * 
     * <p>Sets all motors to the configured hold speed, typically used
     * to maintain position against gravity or other forces.
     */
    public void stop() {
        setAllMotors(holdSpeed);
    }

    /**
     * Updates motor speeds based on current position and target.
     * 
     * <p>This method implements the position control logic:
     * <ol>
     *   <li>If within threshold of target, stops motors and finishes
     *   <li>If at upper limit, only allows downward movement
     *   <li>If at lower limit, only allows upward movement
     *   <li>Otherwise, moves toward target at configured speed
     * </ol>
     * 
     * <p>This method is called repeatedly by the command returned from {@link #move(double)}.
     */
    private void updateMotorSpeed() {
        double currentValue = currentValueSupplier.get();

        // 1. Within threshold of target. Terminates command.
        if (Math.abs(currentValue - targetValue) <= threshold) {
            setAllMotors(0.0);
            isFinishedToggle = true;
            return;
        }

        boolean needToMoveUp = currentValue < targetValue;
        boolean needToMoveDown = currentValue > targetValue;

        // 2. At or above upper limit. Move down only.
        if (currentValue >= maxValue) {
            if (needToMoveDown) {
                setAllMotors(-motorSpeed);
            } else {
                // Want to move up but at max limit. Stop and finish
                setAllMotors(holdSpeed);
                isFinishedToggle = true;
            }
            return;
        }

        // 3. At or below lower limit. Move up only.
        if (currentValue <= minValue) {
            if (needToMoveUp) {
                setAllMotors(motorSpeed);
            } else {
                // Want to move down but at min limit - stop and finish
                setAllMotors(holdSpeed);
                isFinishedToggle = true;
            }
            return;
        }

        // 4. Within normal range. Move toward target.
        if (needToMoveDown) {
            setAllMotors(-motorSpeed);
        } else {
            setAllMotors(motorSpeed);
        }
    }

    /**
     * Sets all controlled motors to the specified speed.
     * 
     * @param speed the speed to apply to all motors
     */
    private void setAllMotors(double speed) {
        // Loops through all motors and sets the hold speed
        for (MotorWrapper motor : motors) {
            motor.set(speed);
        }
    }
}
