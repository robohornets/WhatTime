package com.btwrobotics.WhatTime.frc.SmartBrake;

import java.util.List;
import java.util.OptionalDouble;

import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manager for intelligent brake control on motor-driven mechanisms.
 * 
 * <p>SmartBrake provides automated position holding by actively controlling motors
 * to maintain a target position. Unlike simple brake mode which relies on motor
 * resistance, SmartBrake actively applies corrective power to counteract external
 * forces and hold position more precisely.
 * 
 * <p>This is particularly useful for mechanisms like elevators or arms that need
 * to hold position against gravity or other continuous forces. The system:
 * <ul>
 *   <li>Monitors current position relative to target
 *   <li>Applies corrective motor power within speed limits
 *   <li>Respects upper and lower position bounds
 *   <li>Uses a threshold to avoid oscillation
 * </ul>
 * 
 * <p>The class also provides methods to enable/disable physical brake mode on
 * the motor controllers.
 * 
 * @see SmartBrakeHelper
 * @see MotorWrapper
 */
public class SmartBrake {
    /** The list of motors controlled by this SmartBrake instance. */
    private final List<MotorWrapper> motors;
    
    /** The target position to maintain when SmartBrake is active. */
    private double targetPosition;
    
    /** The maximum speed allowed for corrective movements. */
    private final double maxSpeed;
    
    /** The tolerance for considering position maintained (deadband). */
    private final double threshold;
    
    /** The maximum allowed position value (upper limit). */
    private final double maxValue;
    
    /** The minimum allowed position value (lower limit). */
    private final double minValue;


    /**
     * Creates a new SmartBrake controller.
     * 
     * @param motors the list of motors to control together
     * @param maxSpeed optional maximum speed for corrections; defaults to 0.3 if empty
     * @param targetPosition the initial target position to maintain
     * @param threshold the tolerance for considering position held
     * @param maxValue the maximum allowed position (upper limit)
     * @param minValue the minimum allowed position (lower limit)
     */
    public SmartBrake(List<MotorWrapper> motors, OptionalDouble maxSpeed, double targetPosition, double threshold, double maxValue, double minValue) {
        this.motors = motors;

        // Sets the custom max speed with a fallback of 0.3.
        this.maxSpeed = maxSpeed.orElse(0.3);

        this.targetPosition = targetPosition;
        this.threshold = threshold;
        this.maxValue = maxValue;
        this.minValue = minValue;
    }

    /**
     * Creates and returns a command that actively maintains the target position.
     * 
     * <p>The returned command will continuously monitor position and apply
     * corrective motor power to hold the mechanism at the target position.
     * This command should be run as a default command for subsystems requiring
     * active position holding.
     * 
     * @return a command that implements the SmartBrake position holding behavior
     * @see SmartBrakeHelper
     */
    public Command enableSmartBrakes() {
        return new SmartBrakeHelper(motors, maxSpeed, targetPosition, threshold, maxValue, minValue);
    }

    /**
     * Updates the target position that SmartBrake will maintain.
     * 
     * <p>Call this method to change where the mechanism should hold position.
     * The active SmartBrake command will automatically adjust to the new target.
     * 
     * @param targetPosition the new target position to maintain
     */
    public void updateBrakePosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    /**
     * Gets the current target position.
     * 
     * @return the target position value
     */
    public double getBrakePositionAsDouble() {
        return targetPosition;
    }

    /**
     * Enables physical brake mode on all motors.
     * 
     * <p>Sets the neutral mode to BRAKE, causing motors to resist movement
     * when no power is applied. This is the traditional brake mode provided
     * by the motor controller hardware.
     */
    public void enableBrakes() {
        for (MotorWrapper motor : motors) {
            motor.getMotor().setNeutralMode(NeutralModeValue.Brake);
        }
    }

    /**
     * Disables physical brake mode on all motors.
     * 
     * <p>Sets the neutral mode to COAST, allowing motors to spin freely
     * when no power is applied. Useful for manual positioning or allowing
     * mechanisms to settle naturally.
     */
    public void disableBrakes() {
        for (MotorWrapper motor : motors) {
            motor.getMotor().setNeutralMode(NeutralModeValue.Coast);
        }
    }
}
