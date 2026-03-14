package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TalonFX wrapper with optional free-drive mode and position control.
 */
public class Motor extends SubsystemBase {
    private static final boolean DEFAULT_INVERTED = false;
    private static final double DEFAULT_MIN_VALUE = -1.0;
    private static final double DEFAULT_MAX_VALUE = 1.0;
    private static final double DEFAULT_MIN_SPEED = 0.0;
    private static final double DEFAULT_MOTOR_SPEED = 1.0;
    private static final boolean DEFAULT_FREE = true;
    private static final double DEFAULT_HOLD_SPEED = 0.0;
    private static final double DEFAULT_THRESHOLD = 0.025;
    private static final double DEFAULT_PG = 0.1;
    private static final DoubleSupplier DEFAULT_POSITION_DOUBLE_SUPPLIER = null;

    private final TalonFX motor;

    private boolean inverted;
    private double minValue;
    private double maxValue;
    private double minSpeed;
    private double motorSpeed;
    private boolean free;
    private double holdSpeed;
    private double threshold;
    private double pG;
    private DoubleSupplier positionSupplier;

    private double targetValue;
    private boolean hasTarget;
    private boolean isHolding;
    private boolean isEnabled;
    private boolean isGoTo;

    @SuppressWarnings("unused")
    private int deccelerateSteps;

    /**
     * Creates a Motor object wrapping a TalonFX motor.
     * 
     * @param deviceId the id assigned to the TalonFX motor
     */
    public Motor(int deviceId) {
        this(deviceId, DEFAULT_INVERTED);
    }

    /**
     * Creates a Motor object wrapping a TalonFX motor.
     * 
     * @param deviceId the id assigned to the TalonFX motor
     * @param inverted whether the motor should be inverted by default
     */
    public Motor(int deviceId, boolean inverted) {
        this(new TalonFX(deviceId), inverted);
    }

    /**
     * Creates a Motor object wrapping a TalonFX motor.
     * 
     * @param deviceId the id assigned to the TalonFX motor
     * @param canbus the name of the canbus the motor is on
     */
    public Motor(int deviceId, String canbus) {
        this(deviceId, canbus, DEFAULT_INVERTED);
    }

    /**
     * Creates a Motor object wrapping a TalonFX motor.
     * 
     * @param deviceId the id assigned to the TalonFX motor
     * @param canbus the name of the canbus the motor is on
     * @param inverted whether the motor should be inverted by default
     */
    public Motor(int deviceId, String canbus, boolean inverted) {
        this(new TalonFX(deviceId, canbus), inverted);
    }

    private Motor(TalonFX motor, boolean inverted) {
        this.motor = motor;
        this.inverted = inverted;

        this.minValue = DEFAULT_MIN_VALUE;
        this.maxValue = DEFAULT_MAX_VALUE;
        this.minSpeed = DEFAULT_MIN_SPEED;
        this.motorSpeed = DEFAULT_MOTOR_SPEED;
        this.free = DEFAULT_FREE;
        this.holdSpeed = DEFAULT_HOLD_SPEED;
        this.threshold = DEFAULT_THRESHOLD;
        this.pG = DEFAULT_PG;
        this.positionSupplier = DEFAULT_POSITION_DOUBLE_SUPPLIER;

        this.targetValue = 0.0;
        this.hasTarget = false;
        this.isHolding = false;
        this.isEnabled = false;
        this.isGoTo = false;

        this.deccelerateSteps = 50;

        setDefaultCommand(Commands.run(this::defaultCommand, this));
    }

    /**
     * Creates a motor from an existing TalonFX object.
     * 
     * @param motor the TalonFX motor
     * @param inverted whether the motor should be inverted by default
     * @return A Motor object that wraps the input TalonFX object
     */
    public static Motor of(TalonFX motor, boolean inverted) {
        return new Motor(motor, inverted);
    }

    public Motor setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public Motor setMinValue(double minValue) {
        validateRange(minValue, this.maxValue);
        this.minValue = minValue;
        return this;
    }

    public Motor setMaxValue(double maxValue) {
        validateRange(this.minValue, maxValue);
        this.maxValue = maxValue;
        return this;
    }

    public Motor setRange(double minValue, double maxValue) {
        validateRange(minValue, maxValue);
        this.minValue = minValue;
        this.maxValue = maxValue;
        return this;
    }

    public Motor setMinSpeed(double minSpeed) {
        validateFinite(minSpeed, "minSpeed");
        validateNonNegative(minSpeed, "minSpeed");
        if (minSpeed > this.motorSpeed) {
            throw new IllegalArgumentException("minSpeed cannot exceed motorSpeed.");
        }
        this.minSpeed = minSpeed;
        return this;
    }

    public Motor setMotorSpeed(double motorSpeed) {
        validateFinite(motorSpeed, "motorSpeed");
        validateNonNegative(motorSpeed, "motorSpeed");
        if (this.minSpeed > motorSpeed) {
            throw new IllegalArgumentException("minSpeed cannot exceed motorSpeed.");
        }
        this.motorSpeed = motorSpeed;
        return this;
    }

    /**
     * Sets the free behaviour of the motor.
     * 
     * @param free whether the motor should move freely
     */
    public Motor setFree(boolean free) {
        this.free = free;
        return this;
    }

    public Motor setHoldSpeed(double holdSpeed) {
        validateFinite(holdSpeed, "holdSpeed");
        this.holdSpeed = holdSpeed;
        return this;
    }

    public Motor setThreshold(double threshold) {
        validateFinite(threshold, "threshold");
        validateNonNegative(threshold, "threshold");
        this.threshold = threshold;
        return this;
    }

    public Motor setPG(double pG) {
        validateFinite(pG, "pG");
        this.pG = pG;
        return this;
    }

    public Motor setPositionSupplier(DoubleSupplier positionSupplier) {
        this.positionSupplier = positionSupplier;
        return this;
    }

    public boolean toggleEnabled() {
        isEnabled = !isEnabled;
        return isEnabled;
    }

    public boolean toggleEnabled(Boolean enabled) {
        if (enabled == null) {
            return toggleEnabled();
        }

        isEnabled = enabled;
        return isEnabled;
    }

    public boolean isEnabled() {
        return isEnabled;
    }

    /**
     * Drive the motor with free rotation at the set motor speed.
     */
    public void drive() {
        drive(motorSpeed);
    }

    /**
     * Drive the motor with free rotation at the set motor speed.
     * 
     * @param reverse whether the direction of the motor should be reversed
     */
    public void drive(boolean reverse) {
        drive(reverse ? -motorSpeed: motorSpeed);
    }

    /**
     * Runs the motor with free rotation at a set speed.
     * 
     * @param speed the speed to run the motor at as a double ranging from -1.0 to 1.0
     */
    public void drive(double speed) {
        if (!free) {
            throw new IllegalStateException(".drive() is disabled; use .goTo() when not using free rotation.");
        }
        if (!isEnabled) {
            warn("Motor is disabled.");
            return;
        }

        if (speed != 0.0 && Math.abs(speed) < minSpeed) {
            warn("Speed too slow (" + Math.abs(speed) + " < " + minSpeed + "), running at minimum speed.");
        }

        targetValue = speed;
        hasTarget = true;
        isHolding = false;
        isGoTo = false;
    }

    public void goTo(double target) {
        if (!isEnabled) {
            warn("Motor is disabled.");
            return;
        }

        targetValue = free ? wrapValue(target) : clamp(target, minValue, maxValue);
        hasTarget = true;
        isHolding = false;
        isGoTo = true;
    }

    /** 
     * Sets the current target position.
     * The motor will move to and hold within the threshold of this position.
     * 
     * @param target the target position for the motor as a double
     */
    public void setTarget(double target) {
        goTo(target);
    }

    /**
     * Sets the neutral mode of the motor
     * 
    *  @param neutralModeValue the neutral mode to apply (BRAKE or COAST)
     * @see NeutralModeValue#Brake
     * @see NeutralModeValue#Coast
     */
    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        motor.setNeutralMode(neutralModeValue);
    }

    public Command brakelessReset(double durationSeconds) {
        if (durationSeconds < 0.0) {
            throw new IllegalArgumentException("duration must be >= 0.");
        }

        return Commands.sequence(
            Commands.runOnce(() -> setNeutralMode(NeutralModeValue.Coast), this),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(() -> setNeutralMode(NeutralModeValue.Brake), this)
        );
    }

    public double update() {
        double speed = 0.0;

        if (!isEnabled || !hasTarget) {
            set(0.0);
            return 0.0;
        }

        if (free && !isGoTo) {
            speed = clamp(targetValue, -1.0, 1.0);
            set(speed);
            return speed;
        }

        double currentValue = getCurrentValue();
        double error = Math.abs(positionError(currentValue, targetValue));

        if (error <= threshold) {
            isHolding = true;
        } else if (error > threshold * 2.0) {
            isHolding = false;
        }

        if (isHolding) {
            speed = holdSpeed;
        } else {
            speed = calculateSpeedWithAcceleration(currentValue);
        }

        if (!free) {
            if (currentValue >= maxValue && speed > 0.0) {
                speed = 0.0;
            }
            if (currentValue <= minValue && speed < 0.0) {
                speed = 0.0;
            }
        }


        set(speed);
        return speed;
    }

    private void defaultCommand() {
        update();
    }

    private double calculateSpeedWithAcceleration(double currentValue) {
        double distanceDifference = positionError(currentValue, targetValue);
        double speed = pG * distanceDifference;

        speed = clamp(speed, -motorSpeed, motorSpeed);

        if (speed != 0.0 && Math.abs(speed) < minSpeed) {
            speed = Math.copySign(minSpeed, speed);
        }

        return speed;
    }

    private double positionError(double currentValue, double targetValue) {
        if (!free) {
            return targetValue - currentValue;
        }

        double span = maxValue - minValue;
        double raw = targetValue - currentValue;
        double shifted = raw + span / 2.0;
        double wrappedShifted = shifted - span * Math.floor(shifted / span);
        return wrappedShifted - span / 2.0;
    }

    private double wrapValue(double value) {
        double span = maxValue - minValue;
        double shifted = value - minValue;
        return shifted - span * Math.floor(shifted / span) + minValue;
    }

    /**
     * Sets the speed of the motor while respecting the motor's inverted setting.
     * 
     * @param speed the speed of the motor as a double from -1.0 to 1.0
     */
    public void set(double speed) {
        double actualSpeed = inverted ? -speed : speed;
        motor.set(actualSpeed);
    }

    public double getCurrentValue() {
        if (positionSupplier != null) {
            return positionSupplier.getAsDouble();
        }
        return getPosition();
    }

    public TalonFX getTalonFX() {
        return motor;
    }

    public TalonFX getMotor() {
        return motor;
    }

    public boolean isInverted() {
        return inverted;
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    private static void validateRange(double minValue, double maxValue) {
        validateFinite(minValue, "minValue");
        validateFinite(maxValue, "maxValue");
        if (maxValue <= minValue) {
            throw new IllegalArgumentException("maxValue must be greater than minValue.");
        }
    }

    private static void validateNonNegative(double value, String name) {
        if (value < 0.0) {
            throw new IllegalArgumentException(name + " must be >= 0.");
        }
    }

    private static void validateFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite.");
        }
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static void warn(String message) {
        DriverStation.reportWarning(message, false);
    }
}
