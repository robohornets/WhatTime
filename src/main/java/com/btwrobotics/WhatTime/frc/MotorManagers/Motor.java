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
    private static final boolean DEFAULT_FREE = false;
    private static final double DEFAULT_HOLD_SPEED = 0.0;
    private static final double DEFAULT_THRESHOLD = 0.025;
    private static final double DEFAULT_PG = 1.0;
    private static final DoubleSupplier DEFAULT_CURRENT_VALUE_SUPPLIER = null;

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
    private DoubleSupplier currentValueSupplier;

    private double targetValue;
    private boolean hasTarget;
    private boolean isHolding;
    private boolean isEnabled;

    @SuppressWarnings("unused")
    private int deccelerateSteps;

    public Motor(TalonFX motor) {
        this(motor, DEFAULT_INVERTED);
    }

    public Motor(TalonFX motor, boolean inverted) {
        this(motor, inverted, DEFAULT_MIN_VALUE);
    }

    public Motor(TalonFX motor, boolean inverted, double minValue) {
        this(motor, inverted, minValue, DEFAULT_MAX_VALUE);
    }

    public Motor(TalonFX motor, boolean inverted, double minValue, double maxValue) {
        this(motor, inverted, minValue, maxValue, DEFAULT_MIN_SPEED);
    }

    public Motor(TalonFX motor, boolean inverted, double minValue, double maxValue, double minSpeed) {
        this(motor, inverted, minValue, maxValue, minSpeed, DEFAULT_MOTOR_SPEED);
    }

    public Motor(TalonFX motor, boolean inverted, double minValue, double maxValue, double minSpeed, double motorSpeed) {
        this(motor, inverted, minValue, maxValue, minSpeed, motorSpeed, DEFAULT_FREE);
    }

    public Motor(
        TalonFX motor,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free
    ) {
        this(motor, inverted, minValue, maxValue, minSpeed, motorSpeed, free, DEFAULT_HOLD_SPEED);
    }

    public Motor(
        TalonFX motor,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed
    ) {
        this(motor, inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed, DEFAULT_THRESHOLD);
    }

    public Motor(
        TalonFX motor,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed,
        double threshold
    ) {
        this(motor, inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed, threshold, DEFAULT_PG);
    }

    public Motor(
        TalonFX motor,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed,
        double threshold,
        double pG
    ) {
        this(motor, inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed, threshold, pG, DEFAULT_CURRENT_VALUE_SUPPLIER);
    }

    public Motor(int deviceId) {
        this(new TalonFX(deviceId), DEFAULT_INVERTED);
    }

    public Motor(int deviceId, boolean inverted) {
        this(new TalonFX(deviceId), inverted);
    }

    public Motor(int deviceId, boolean inverted, double minValue) {
        this(new TalonFX(deviceId), inverted, minValue);
    }

    public Motor(int deviceId, boolean inverted, double minValue, double maxValue) {
        this(new TalonFX(deviceId), inverted, minValue, maxValue);
    }

    public Motor(int deviceId, boolean inverted, double minValue, double maxValue, double minSpeed) {
        this(new TalonFX(deviceId), inverted, minValue, maxValue, minSpeed);
    }

    public Motor(int deviceId, boolean inverted, double minValue, double maxValue, double minSpeed, double motorSpeed) {
        this(new TalonFX(deviceId), inverted, minValue, maxValue, minSpeed, motorSpeed);
    }

    public Motor(
        int deviceId,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free
    ) {
        this(new TalonFX(deviceId), inverted, minValue, maxValue, minSpeed, motorSpeed, free);
    }

    public Motor(
        int deviceId,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed
    ) {
        this(new TalonFX(deviceId), inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed);
    }

    public Motor(
        int deviceId,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed,
        double threshold
    ) {
        this(new TalonFX(deviceId), inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed, threshold);
    }

    public Motor(
        int deviceId,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed,
        double threshold,
        double pG
    ) {
        this(new TalonFX(deviceId), inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed, threshold, pG);
    }

    public Motor(
        int deviceId,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed,
        double threshold,
        double pG,
        DoubleSupplier currentValueSupplier
    ) {
        this(new TalonFX(deviceId), inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed, threshold, pG, currentValueSupplier);
    }

    public Motor(int deviceId, String canbus) {
        this(new TalonFX(deviceId, canbus), DEFAULT_INVERTED);
    }

    public Motor(int deviceId, String canbus, boolean inverted) {
        this(new TalonFX(deviceId, canbus), inverted);
    }

    public Motor(int deviceId, String canbus, boolean inverted, double minValue) {
        this(new TalonFX(deviceId, canbus), inverted, minValue);
    }

    public Motor(int deviceId, String canbus, boolean inverted, double minValue, double maxValue) {
        this(new TalonFX(deviceId, canbus), inverted, minValue, maxValue);
    }

    public Motor(int deviceId, String canbus, boolean inverted, double minValue, double maxValue, double minSpeed) {
        this(new TalonFX(deviceId, canbus), inverted, minValue, maxValue, minSpeed);
    }

    public Motor(int deviceId, String canbus, boolean inverted, double minValue, double maxValue, double minSpeed, double motorSpeed) {
        this(new TalonFX(deviceId, canbus), inverted, minValue, maxValue, minSpeed, motorSpeed);
    }

    public Motor(
        int deviceId,
        String canbus,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free
    ) {
        this(new TalonFX(deviceId, canbus), inverted, minValue, maxValue, minSpeed, motorSpeed, free);
    }

    public Motor(
        int deviceId,
        String canbus,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed
    ) {
        this(new TalonFX(deviceId, canbus), inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed);
    }

    public Motor(
        int deviceId,
        String canbus,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed,
        double threshold
    ) {
        this(new TalonFX(deviceId, canbus), inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed, threshold);
    }

    public Motor(
        int deviceId,
        String canbus,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed,
        double threshold,
        double pG
    ) {
        this(new TalonFX(deviceId, canbus), inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed, threshold, pG);
    }

    public Motor(
        int deviceId,
        String canbus,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed,
        double threshold,
        double pG,
        DoubleSupplier currentValueSupplier
    ) {
        this(new TalonFX(deviceId, canbus), inverted, minValue, maxValue, minSpeed, motorSpeed, free, holdSpeed, threshold, pG, currentValueSupplier);
    }

    public Motor(
        TalonFX motor,
        boolean inverted,
        double minValue,
        double maxValue,
        double minSpeed,
        double motorSpeed,
        boolean free,
        double holdSpeed,
        double threshold,
        double pG,
        DoubleSupplier currentValueSupplier
    ) {
        validateRange(minValue, maxValue);
        validateSpeeds(motorSpeed, minSpeed, holdSpeed);
        validateFinite(threshold, "threshold");
        validateNonNegative(threshold, "threshold");
        validateFinite(pG, "pG");

        this.motor = motor;
        this.inverted = inverted;

        this.minValue = minValue;
        this.maxValue = maxValue;
        this.minSpeed = minSpeed;
        this.motorSpeed = motorSpeed;
        this.free = free;
        this.holdSpeed = holdSpeed;
        this.threshold = threshold;
        this.pG = pG;
        this.currentValueSupplier = currentValueSupplier;

        this.targetValue = 0.0;
        this.hasTarget = false;
        this.isHolding = false;
        this.isEnabled = false;

        this.deccelerateSteps = 50;

        setDefaultCommand(Commands.run(this::defaultCommand, this));
    }

    public static Motor of(TalonFX motor, boolean inverted) {
        return new Motor(motor, inverted);
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

    public void config(Map<String, ?> config) {
        for (Map.Entry<String, ?> entry : config.entrySet()) {
            String key = entry.getKey();
            Object value = entry.getValue();

            switch (key) {
                case "inverted":
                    if (value instanceof Boolean boolValue) {
                        inverted = boolValue;
                    } else {
                        warn("Invalid type for 'inverted', expected boolean.");
                    }
                    break;
                case "minValue":
                    if (value instanceof Number number) {
                        double next = number.doubleValue();
                        validateRange(next, maxValue);
                        minValue = next;
                    } else {
                        warn("Invalid type for 'minValue', expected number.");
                    }
                    break;
                case "maxValue":
                    if (value instanceof Number number) {
                        double next = number.doubleValue();
                        validateRange(minValue, next);
                        maxValue = next;
                    } else {
                        warn("Invalid type for 'maxValue', expected number.");
                    }
                    break;
                case "minSpeed":
                    if (value instanceof Number number) {
                        double next = number.doubleValue();
                        validateFinite(next, "minSpeed");
                        validateNonNegative(next, "minSpeed");
                        if (next > motorSpeed) {
                            throw new IllegalArgumentException("minSpeed cannot exceed motorSpeed.");
                        }
                        minSpeed = next;
                    } else {
                        warn("Invalid type for 'minSpeed', expected number.");
                    }
                    break;
                case "motorSpeed":
                    if (value instanceof Number number) {
                        double next = number.doubleValue();
                        validateFinite(next, "motorSpeed");
                        validateNonNegative(next, "motorSpeed");
                        if (minSpeed > next) {
                            throw new IllegalArgumentException("minSpeed cannot exceed motorSpeed.");
                        }
                        motorSpeed = next;
                    } else {
                        warn("Invalid type for 'motorSpeed', expected number.");
                    }
                    break;
                case "free":
                    if (value instanceof Boolean boolValue) {
                        free = boolValue;
                    } else {
                        warn("Invalid type for 'free', expected boolean.");
                    }
                    break;
                case "holdSpeed":
                    if (value instanceof Number number) {
                        double next = number.doubleValue();
                        validateFinite(next, "holdSpeed");
                        holdSpeed = next;
                    } else {
                        warn("Invalid type for 'holdSpeed', expected number.");
                    }
                    break;
                case "threshold":
                    if (value instanceof Number number) {
                        double next = number.doubleValue();
                        validateFinite(next, "threshold");
                        validateNonNegative(next, "threshold");
                        threshold = next;
                    } else {
                        warn("Invalid type for 'threshold', expected number.");
                    }
                    break;
                case "pG":
                    if (value instanceof Number number) {
                        double next = number.doubleValue();
                        validateFinite(next, "pG");
                        pG = next;
                    } else {
                        warn("Invalid type for 'pG', expected number.");
                    }
                    break;
                case "currentValueSupplier":
                    if (value == null || value instanceof DoubleSupplier supplier) {
                        currentValueSupplier = (DoubleSupplier) value;
                    } else {
                        warn("Invalid type for 'currentValueSupplier', expected DoubleSupplier.");
                    }
                    break;
                case "deccelerateSteps":
                    if (value instanceof Number number) {
                        deccelerateSteps = number.intValue();
                    } else {
                        warn("Invalid type for 'deccelerateSteps', expected number.");
                    }
                    break;
                default:
                    warn("Motor has no property '" + key + "', skipping");
                    break;
            }
        }
    }

    public void drive() {
        drive(motorSpeed);
    }

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
    }

    public void goTo(double target) {
        if (!isEnabled) {
            warn("Motor is disabled.");
            return;
        }

        targetValue = free ? wrapValue(target) : clamp(target, minValue, maxValue);
        hasTarget = true;
        isHolding = false;
    }

    public void setTarget(double target) {
        goTo(target);
    }

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

        if (free) {
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

        if (currentValue >= maxValue && speed > 0.0) {
            speed = 0.0;
        }
        if (currentValue <= minValue && speed < 0.0) {
            speed = 0.0;
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

    public void set(double speed) {
        double actualSpeed = inverted ? -speed : speed;
        motor.set(actualSpeed);
    }

    public double getCurrentValue() {
        if (currentValueSupplier != null) {
            return currentValueSupplier.getAsDouble();
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

    private static void validateSpeeds(double motorSpeed, double minSpeed, double holdSpeed) {
        validateFinite(motorSpeed, "motorSpeed");
        validateFinite(minSpeed, "minSpeed");
        validateFinite(holdSpeed, "holdSpeed");
        validateNonNegative(motorSpeed, "motorSpeed");
        validateNonNegative(minSpeed, "minSpeed");
        if (minSpeed > motorSpeed) {
            throw new IllegalArgumentException("minSpeed cannot exceed motorSpeed.");
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
