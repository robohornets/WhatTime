package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Deprecated: Use Motor class instead
 * 
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

@Deprecated
public class PositionManager extends SubsystemBase {
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

    /** The lowest speed allowed for the motor to prevent stalling. */
    private final double minSpeed;

    /** Kim Possible - A tuning constant for the acceleration calculation of the motor. */
    public final double kP;

    /** Supplier that provides the current position value. */
    private final Supplier<Double> currentValueSupplier;
    
    /** The current target position for the motors to reach. */
    private double targetValue;

    /** Whether a target has been explicitly set. Until set, motors stay put. */
    private boolean hasTarget = false;

    /** Whether the mechanism is currently holding at the target position. */
    private boolean isHolding = false;
    
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
        double minSpeed,
        double kP,
        Supplier<Double> currentValueSupplier
    ) {
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.motors = motors;
        this.targetValue = minValue;
        this.motorSpeed = motorSpeed;
        this.holdSpeed = holdSpeed;
        this.threshold = threshold;
        this.minSpeed = minSpeed;
        this.kP = kP;
        this.currentValueSupplier = currentValueSupplier;

        setDefaultCommand(positionTargetManagement());
    }

    public boolean isDisabled = false;

    public void toggleDisabled() {
        isDisabled = !isDisabled;
    }

    /** 
     * Sets the current target position
     * 
     * The motor will move to and hold within the threshold of this position
     */
    public void setTarget(double target) {
        this.targetValue = Math.max(minValue, Math.min(maxValue, target));
        this.hasTarget = true;
        this.isHolding = false;
    }

    private Command positionTargetManagement() {
        return Commands.run(() -> {
            if (isDisabled) {
                setAllMotors(0);
                return;
            }
            if (!hasTarget) {
                setAllMotors(0);
                return;
            }

            double currentValue = currentValueSupplier.get();
            double error = Math.abs(currentValue - targetValue);

            if (error <= threshold) {
                isHolding = true;
            } else if (error > threshold * 2) {
                isHolding = false;
            }

            if (isHolding) {
                double speed = holdSpeed;
                if (currentValue >= maxValue && speed > 0) speed = 0;
                if (currentValue <= minValue && speed < 0) speed = 0;
                setAllMotors(speed);
                return;
            }

            double speed = calculateSpeedWithAcceleration();

            if (currentValue >= maxValue && speed > 0) speed = 0;
            if (currentValue <= minValue && speed < 0) speed = 0;

            setAllMotors(speed);
            
        }, this);
    }
    
    /**
     * Gets the current target position.
     * 
     * @return the target position value
     */
    public double getTarget() {
        return targetValue;
    }

    
    private double calculateSpeedWithAcceleration() {
        double distanceDifference = getTarget() - currentValueSupplier.get();
        
        double speed = kP * distanceDifference;

        // Clamp to prevent stalling at low values
        speed = Math.max(-motorSpeed, Math.min(motorSpeed, speed));

        if (Math.abs(speed) < minSpeed) {
            speed = Math.copySign(minSpeed, speed);
        }
        return speed;
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
