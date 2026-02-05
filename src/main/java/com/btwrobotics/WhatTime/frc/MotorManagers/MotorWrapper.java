package com.btwrobotics.WhatTime.frc.MotorManagers;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Wrapper class for TalonFX motor controllers with inversion support.
 * 
 * <p>This class provides a simplified interface for controlling TalonFX motors
 * with built-in support for motor inversion. The inversion is handled at the
 * wrapper level, allowing for consistent speed control regardless of the motor's
 * physical mounting orientation.
 * 
 * <p>This wrapper is particularly useful when working with motor groups where
 * some motors may be mounted in opposite directions but should respond to
 * commands uniformly.
 * 
 * @see com.ctre.phoenix6.hardware.TalonFX
 */
public class MotorWrapper {
    /** The underlying TalonFX motor controller. */
    private final TalonFX motor;
    
    /** Whether this motor's output should be inverted. */
    private final boolean inverted;

    /**
     * Creates a new MotorWrapper.
     * 
     * @param motor the TalonFX motor controller to wrap
     * @param inverted true to invert the motor's direction, false for normal operation
     */
    public MotorWrapper(TalonFX motor, boolean inverted) {
        this.motor = motor;
        this.inverted = inverted;
    }

    /**
     * Factory method to create a MotorWrapper using fluent syntax.
     * 
     * <p>This method provides an alternative to using the constructor directly,
     * enabling a more fluent coding style with {@code MotorWrapper.of()}.
     * 
     * @param motor the TalonFX motor controller to wrap
     * @param inverted true to invert the motor's direction, false for normal operation
     * @return a new MotorWrapper instance
     */
    public static MotorWrapper of(TalonFX motor, boolean inverted) {
        return new MotorWrapper(motor, inverted);
    }

    /**
     * Sets the motor speed, automatically handling inversion.
     * 
     * <p>If the motor is marked as inverted, the speed will be negated before
     * being sent to the motor controller. This allows for consistent control
     * regardless of physical motor orientation.
     * 
     * @param speed the desired speed, typically in the range [-1.0, 1.0]
     */
    public void set(double speed) {
        double actualSpeed = inverted ? -speed : speed;
        motor.set(actualSpeed);
    }

    /**
     * Gets the underlying TalonFX motor controller.
     * 
     * <p>Use this method to access advanced motor controller features that
     * are not exposed by the wrapper.
     * 
     * @return the TalonFX motor controller instance
     */
    public TalonFX getMotor() {
        return motor;
    }

    /**
     * Checks if the motor output is inverted.
     * 
     * <p>This information can be used by other classes to determine the
     * expected rotation direction of the motor.
     * 
     * @return true if the motor is inverted, false otherwise
     */
    public boolean isInverted() {
        return inverted;
    }

    /**
     * Gets the current motor position.
     * 
     * <p>Returns the motor's current output value (duty cycle), not the encoder position.
     * 
     * @return the current motor output value
     */
    public double getPosition() {
        return motor.get();
    }
}
