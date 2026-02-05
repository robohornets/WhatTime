package com.btwrobotics.WhatTime.frc.SmartBrake;

import java.util.List;

import com.btwrobotics.WhatTime.frc.MotorManagers.MotorPositionHandler;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command implementation for SmartBrake position holding with proportional control.
 * 
 * <p>This command implements the active position holding logic for SmartBrake.
 * It continuously monitors the average position of controlled motors and applies
 * proportional corrective power to maintain the target position.
 * 
 * <p>The control algorithm uses distance-based proportional control:
 * <ul>
 *   <li>Within threshold: No corrective action needed
 *   <li>Below target: Apply upward force proportional to distance from minimum
 *   <li>Above target: Apply downward force proportional to distance from maximum
 * </ul>
 * 
 * <p>A small constant (0.05) is added to overcome static friction and ensure movement.
 * 
 * @see SmartBrake
 * @see MotorPositionHandler
 */
public class SmartBrakeHelper extends Command {
    /** The list of motors to control for position holding. */
    private final List<MotorWrapper> motors;
    
    /** The maximum speed allowed for corrective movements. */
    private final double maxSpeed;
    
    /** The target position to maintain. */
    private final double targetPosition;
    
    /** The tolerance for considering position maintained (deadband). */
    private final double threshold;
    
    /** The maximum allowed position value (upper limit). */
    private final double maxValue;
    
    /** The minimum allowed position value (lower limit). */
    private final double minValue;

    /**
     * Creates a new SmartBrakeHelper command.
     * 
     * @param motors the list of motors to control together
     * @param maxSpeed the maximum speed for corrective movements
     * @param targetPosition the position to maintain
     * @param threshold the tolerance for position holding
     * @param maxValue the maximum allowed position (upper limit)
     * @param minValue the minimum allowed position (lower limit)
     */
    public SmartBrakeHelper(List<MotorWrapper> motors, double maxSpeed, double targetPosition, double threshold, double maxValue, double minValue) {
        this.motors = motors;
        this.maxSpeed = maxSpeed;
        this.targetPosition = targetPosition;
        this.threshold = threshold;
        this.maxValue = maxValue;
        this.minValue = minValue;
    }

    /** Internal flag for command completion status. */
    private boolean isFinishedToggle = false;

    /** Handler for calculating average motor positions. */
    private MotorPositionHandler motorPositionHandler = new MotorPositionHandler();

    /**
     * Initializes the command by resetting the finished flag.
     * 
     * <p>Called when the command is first scheduled.
     */
    @Override
    public void initialize() {
        isFinishedToggle = false;
    }

    /**
     * Executes the position holding control logic.
     * 
     * <p>Called repeatedly while the command is scheduled. This method:
     * <ol>
     *   <li>Calculates the average position of all motors
     *   <li>If within threshold, applies no correction
     *   <li>If below target, applies upward force proportional to distance from minimum
     *   <li>If above target, applies downward force proportional to distance from maximum
     * </ol>
     * 
     * <p>The proportional calculation scales motor speed based on how far the
     * mechanism is from its limit, with a small constant offset to overcome friction.
     */
    @Override
    public void execute() {
        // If the position is correct, do nothing
        if (Math.abs(targetPosition - motorPositionHandler.averagePositions(motors)) <= threshold) {
        }

        // Calculate percentage of movement
        else {
            // Below target
            if (motorPositionHandler.averagePositions(motors) < targetPosition) {
                double percentageMoved = Math.abs(motorPositionHandler.averagePositions(motors) / (minValue - targetPosition));

                setAllMotors(percentageMoved * maxSpeed + 0.05);
            }
            // Above target
            else {
                double percentageMoved = Math.abs(motorPositionHandler.averagePositions(motors) / (maxValue - targetPosition));

                setAllMotors(-percentageMoved * maxSpeed + 0.05);
            }
        }
    }

    /**
     * Determines whether the command has finished executing.
     * 
     * <p>This command is designed to run indefinitely as a default command,
     * so it typically returns false. The finished toggle can be used to
     * terminate the command if needed.
     * 
     * @return true if the command should finish, false otherwise
     */
    @Override
    public boolean isFinished() {
        return isFinishedToggle;
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
