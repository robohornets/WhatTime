package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Utility class for performing bulk operations on multiple motors.
 * 
 * <p>This class provides methods to apply configuration changes to multiple
 * {@link MotorWrapper} instances simultaneously, reducing code duplication
 * and improving maintainability when working with groups of motors.
 * 
 * @see MotorWrapper
 * @see com.ctre.phoenix6.signals.NeutralModeValue
 */
public class MotorBulkActions {
    /**
     * Sets the neutral mode for multiple motors.
     * 
     * <p>Applies the specified neutral mode (brake or coast) to all motors in the list.
     * This is useful for coordinating behavior across multiple motors in a subsystem,
     * such as setting all motors to brake mode for holding position or coast mode
     * for free movement.
     * 
     * @param motors the list of motor wrappers to configure
     * @param neutralModeValue the neutral mode to apply (BRAKE or COAST)
     * @see NeutralModeValue#Brake
     * @see NeutralModeValue#Coast
     */
    public void setNeutralModeBulk(List<MotorWrapper> motors, NeutralModeValue neutralModeValue) {
        for (MotorWrapper motor : motors) {
            motor.getMotor().setNeutralMode(neutralModeValue);
        }
    }
}
