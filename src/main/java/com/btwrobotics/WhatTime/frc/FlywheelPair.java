package com.btwrobotics.WhatTime.frc;

import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;

/**
 * Coordinated control for a pair of motors operating as a synchronized unit.
 * 
 * <p>This class manages two motors that work together, primarily designed for
 * dual-motor intake/shooter systems or roller mechanisms. It ensures both motors
 * run at the same speed in the same direction, automatically handling motor
 * inversion based on physical mounting orientation.
 * 
 * <p>The class provides:
 * <ul>
 *   <li>Synchronized forward and backward operation
 *   <li>Default speed configuration with override capability
 *   <li>Automatic inversion handling for proper directional control
 *   <li>Simple stop functionality for both motors
 * </ul>
 * 
 * @see MotorWrapper
 */
public class FlywheelPair {
    /** The first motor in the pair. */
    private MotorWrapper motor1;
    
    /** The second motor in the pair. */
    private MotorWrapper motor2;
    
    /** The default speed for motor operations. */
    private double speed;

    /**
     * Creates a new FlywheelPair with two motors and a default speed.
     * 
     * @param motor1 the first motor in the pair
     * @param motor2 the second motor in the pair
     * @param speed the default speed for operations (typically 0.0 to 1.0)
     */
    public FlywheelPair(
        MotorWrapper motor1,
        MotorWrapper motor2,
        double speed
    ) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.speed = speed;
    }

    /**
     * Runs both motors forward at the default class speed.
     * 
     * <p>Forward direction is determined by the configured default speed
     * and automatically accounts for motor inversion.
     */
    public void runForward() {
        runForward(speed);
    }

    /**
     * Runs both motors forward at a specified speed.
     * 
     * <p>The specified speed overrides the default class speed for this
     * operation. Motor inversion is automatically handled to ensure both
     * motors spin in the correct direction.
     * 
     * @param overrideSpeed the speed to run at, overriding the default
     */
    public void runForward(double overrideSpeed) {
        motor1.set(overrideSpeed * (motor1.isInverted() ? -1 : 1));
        motor2.set(overrideSpeed * (motor2.isInverted() ? -1 : 1));
    }

    /**
     * Runs both motors backward at the default class speed.
     * 
     * <p>Backward direction is opposite to forward, with automatic handling
     * of motor inversion for correct directional control.
     */
    public void runBackward() {
        runBackward(speed);
    }

    /**
     * Runs both motors backward at a specified speed.
     * 
     * <p>The specified speed overrides the default class speed for this
     * operation. Motor inversion is automatically handled to ensure both
     * motors spin in the correct reverse direction.
     * 
     * @param overrideSpeed the speed to run at, overriding the default
     */
    public void runBackward(double overrideSpeed) {
        motor1.set(overrideSpeed * (motor1.isInverted() ? 1 : -1));
        motor2.set(overrideSpeed * (motor2.isInverted() ? 1 : -1));
    }

    /**
     * Stops both motors by setting their speed to zero.
     * 
     * <p>This immediately halts both motors. The motors will coast or brake
     * depending on their configured neutral mode.
     */
    public void stopMotors() {
        motor1.set(0);
        motor2.set(0);
    }
}
