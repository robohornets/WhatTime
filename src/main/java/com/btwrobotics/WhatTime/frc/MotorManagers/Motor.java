package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A TalonFX that either drives at a speed or moves to and holds a position.
 *
 * <p>The motor updates itself every loop, so one call is enough: {@code drive(0.5)} keeps
 * spinning until {@link #stop()}, and {@code goTo(30)} moves there and holds. The motor stops
 * whenever the robot is disabled.
 *
 * <pre>{@code
 * Motor intake = new Motor(10);
 * controller.a().whileTrue(intake.driveCommand(0.8)); // runs while held, stops on release
 *
 * Motor elevator = new Motor(11).setRange(0, 50).setMotorSpeed(0.6).setAccelerationTime(0.3);
 * controller.b().onTrue(elevator.goToCommand(30));    // moves to 30 and holds there
 * }</pre>
 */
public class Motor extends SubsystemBase {
    private final TalonFX motor;

    private boolean inverted;
    private boolean enabled = true;
    private double minValue = Double.NEGATIVE_INFINITY;
    private double maxValue = Double.POSITIVE_INFINITY;
    /** Length of one full turn when continuous, 0 when not. */
    private double fullTurn = 0.0;
    private double minSpeed = 0.0;
    private double motorSpeed = 1.0;
    private Double motorUpSpeed;
    private Double motorDownSpeed;
    private double holdSpeed = 0.0;
    private double threshold = 0.025;
    private double kP = 0.1;
    /** Limits how fast the output can change, or null when acceleration is off. */
    private SlewRateLimiter accelerationLimiter;
    private DoubleSupplier positionSupplier;

    /** Speed to drive at when there is no position target. */
    private double driveSpeed;
    /** Position to move to and hold, or null when driving. */
    private Double target;
    private boolean holding;
    /** Output sent to the motor last loop. */
    private double output;

    /**
     * Creates a Motor object wrapping a TalonFX motor.
     *
     * @param deviceId the id assigned to the TalonFX motor
     */
    public Motor(int deviceId) {
        this(deviceId, false);
    }

    /**
     * Creates a Motor object wrapping a TalonFX motor.
     *
     * @param deviceId the id assigned to the TalonFX motor
     * @param inverted whether the motor should be inverted
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
        this(deviceId, canbus, false);
    }

    /**
     * Creates a Motor object wrapping a TalonFX motor.
     *
     * @param deviceId the id assigned to the TalonFX motor
     * @param canbus the name of the canbus the motor is on
     * @param inverted whether the motor should be inverted
     */
    public Motor(int deviceId, String canbus, boolean inverted) {
        this(new TalonFX(deviceId, new CANBus(canbus)), inverted);
    }

    private Motor(TalonFX motor, boolean inverted) {
        this.motor = motor;
        this.inverted = inverted;
    }

    /**
     * Creates a motor from an existing TalonFX object.
     *
     * @param motor the TalonFX motor
     * @param inverted whether the motor should be inverted
     * @return A Motor object that wraps the input TalonFX object
     */
    public static Motor of(TalonFX motor, boolean inverted) {
        return new Motor(motor, inverted);
    }

    /**
     * Flips the motor's direction, so positive speeds spin it the other way. The TalonFX's
     * built-in encoder is flipped to match, so positions still go up when driving forward.
     *
     * @param inverted whether the motor should be inverted
     * @return this motor, for chaining
     */
    public Motor setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    /**
     * Sets just the lower soft limit. See {@link #setRange}.
     *
     * @param minValue the lowest position the motor may drive to
     * @return this motor, for chaining
     */
    public Motor setMinValue(double minValue) {
        return setRange(minValue, maxValue);
    }

    /**
     * Sets just the upper soft limit. See {@link #setRange}.
     *
     * @param maxValue the highest position the motor may drive to
     * @return this motor, for chaining
     */
    public Motor setMaxValue(double maxValue) {
        return setRange(minValue, maxValue);
    }

    /**
     * Sets soft limits. The motor won't drive further past them, and {@link #goTo} targets are
     * clamped into them. With acceleration on, the motor only starts slowing down once it reaches
     * a limit, so leave some room before any hard stop. There are no limits by default.
     *
     * @param minValue the lowest position the motor may drive to
     * @param maxValue the highest position the motor may drive to
     * @return this motor, for chaining
     */
    public Motor setRange(double minValue, double maxValue) {
        if (!(maxValue > minValue)) {
            throw new IllegalArgumentException("maxValue must be greater than minValue.");
        }
        this.minValue = minValue;
        this.maxValue = maxValue;
        return this;
    }

    /**
     * For mechanisms that spin all the way around, like a turret. {@code min} and {@code max} are
     * the same spot, and {@link #goTo} takes the shortest way to the target: with
     * {@code setContinuous(0, 1)}, going from 0.9 to 0.1 moves forward 0.2 instead of back 0.8.
     * Works no matter how many times the mechanism has spun around. Don't combine with soft
     * limits, since the shortest way may cross them.
     *
     * @param min the start of one full turn, e.g. 0
     * @param max the end of one full turn, e.g. 1 for rotations or 360 for degrees
     * @return this motor, for chaining
     */
    public Motor setContinuous(double min, double max) {
        if (!(max > min)) {
            throw new IllegalArgumentException("max must be greater than min.");
        }
        this.fullTurn = max - min;
        return this;
    }

    /**
     * Sets the smallest non-zero speed the motor will run at, so it doesn't stall on friction
     * when it's close to a target. Must not be more than motorSpeed.
     *
     * @param minSpeed from 0.0 to motorSpeed
     * @return this motor, for chaining
     */
    public Motor setMinSpeed(double minSpeed) {
        this.minSpeed = check(minSpeed, 0.0, motorSpeed, "minSpeed");
        return this;
    }

    /**
     * Sets the speed {@link #drive()} runs at, and the top speed when moving with {@link #goTo}.
     *
     * @param motorSpeed from minSpeed to 1.0
     * @return this motor, for chaining
     */
    public Motor setMotorSpeed(double motorSpeed) {
        this.motorSpeed = check(motorSpeed, minSpeed, 1.0, "motorSpeed");
        return this;
    }

    /**
     * Sets the top {@link #goTo} speed when moving toward higher positions, e.g. a slower speed
     * for lifting an elevator.
     *
     * @param motorUpSpeed from minSpeed to 1.0, or {@code null} to use motorSpeed
     * @return this motor, for chaining
     */
    public Motor setMotorUpSpeed(Double motorUpSpeed) {
        this.motorUpSpeed = motorUpSpeed == null ? null : check(motorUpSpeed, minSpeed, 1.0, "motorUpSpeed");
        return this;
    }

    /**
     * Sets the top {@link #goTo} speed when moving toward lower positions, e.g. a slower speed
     * so an elevator doesn't slam down.
     *
     * @param motorDownSpeed from minSpeed to 1.0, or {@code null} to use motorSpeed
     * @return this motor, for chaining
     */
    public Motor setMotorDownSpeed(Double motorDownSpeed) {
        this.motorDownSpeed = motorDownSpeed == null ? null : check(motorDownSpeed, minSpeed, 1.0, "motorDownSpeed");
        return this;
    }

    /**
     * Makes the motor change speed gradually instead of jumping. This applies to every change:
     * speeding up, slowing down, stopping and reversing (which slows to zero first). The only
     * exception is the robot being disabled, which cuts power right away no matter what.
     *
     * @param seconds time to go from stopped to full speed (or full speed to stopped);
     *     0, the default, turns acceleration off
     * @return this motor, for chaining
     */
    public Motor setAccelerationTime(double seconds) {
        check(seconds, 0.0, Double.POSITIVE_INFINITY, "accelerationTime");
        accelerationLimiter = seconds > 0.0 ? new SlewRateLimiter(1.0 / seconds, -1.0 / seconds, output) : null;
        return this;
    }

    /**
     * Sets the output used once the motor reaches its {@link #goTo} target, e.g. a little
     * upward power to hold an elevator against gravity. Defaults to 0.
     *
     * @param holdSpeed from -1.0 to 1.0
     * @return this motor, for chaining
     */
    public Motor setHoldSpeed(double holdSpeed) {
        this.holdSpeed = check(holdSpeed, -1.0, 1.0, "holdSpeed");
        return this;
    }

    /**
     * Sets how close to the {@link #goTo} target counts as being there. Once inside it the motor
     * switches to holdSpeed, and it only starts correcting again after drifting twice as far.
     *
     * @param threshold the allowed distance from the target, in position units
     * @return this motor, for chaining
     */
    public Motor setThreshold(double threshold) {
        this.threshold = check(threshold, 0.0, Double.POSITIVE_INFINITY, "threshold");
        return this;
    }

    /**
     * Sets how hard {@link #goTo} pushes toward the target: speed = kP * distance to target,
     * capped at the top speed. Higher gets there faster but can overshoot.
     *
     * @param kP the proportional gain, 0 or more
     * @return this motor, for chaining
     */
    public Motor setKP(double kP) {
        this.kP = check(kP, 0.0, Double.POSITIVE_INFINITY, "kP");
        return this;
    }

    /**
     * Reads the position from somewhere other than the TalonFX's built-in encoder, e.g. a
     * through-bore or CANcoder. This is used as-is, without flipping for inversion.
     *
     * @param positionSupplier where to read the position from, or {@code null} for the built-in encoder
     * @return this motor, for chaining
     */
    public Motor setPositionSupplier(DoubleSupplier positionSupplier) {
        this.positionSupplier = positionSupplier;
        return this;
    }

    /**
     * Enables or disables the motor. Disabling stops it and ignores {@link #drive} and
     * {@link #goTo} until it's enabled again. Motors start enabled.
     *
     * @param enabled whether the motor should be enabled
     * @return this motor, for chaining
     */
    public Motor setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            stop();
        }
        return this;
    }

    /**
     * Flips between enabled and disabled. See {@link #setEnabled}.
     *
     * @return whether the motor is now enabled
     */
    public boolean toggleEnabled() {
        setEnabled(!enabled);
        return enabled;
    }

    /**
     * Whether the motor is enabled. See {@link #setEnabled}.
     *
     * @return whether the motor is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /** Drives forward at motorSpeed until stopped or told to do something else. */
    public void drive() {
        drive(motorSpeed);
    }

    /**
     * Drives at motorSpeed until stopped or told to do something else.
     *
     * @param reverse whether to drive backwards
     */
    public void drive(boolean reverse) {
        drive(reverse ? -motorSpeed : motorSpeed);
    }

    /**
     * Drives at {@code speed} until stopped or told to do something else. Clears any
     * {@link #goTo} target.
     *
     * @param speed from -1.0 to 1.0, negative for backwards
     */
    public void drive(double speed) {
        if (!enabled) {
            return;
        }
        target = null;
        driveSpeed = applyMinSpeed(MathUtil.clamp(speed, -1.0, 1.0));
    }

    /**
     * Moves to {@code position} and holds it there until stopped or told to do something else.
     * The position is clamped into the soft limits.
     *
     * @param position the position to move to, in the same units as {@link #getPosition}
     */
    public void goTo(double position) {
        if (!enabled) {
            return;
        }
        target = MathUtil.clamp(position, minValue, maxValue);
        holding = false;
    }

    /**
     * Same as {@link #goTo}.
     *
     * @param position the position to move to
     */
    public void setTarget(double position) {
        goTo(position);
    }

    /** Stops driving and clears any target. With acceleration on, the motor slows down to a stop. */
    public void stop() {
        target = null;
        driveSpeed = 0.0;
    }

    /**
     * Whether the motor has a {@link #goTo} target and is within the threshold of it.
     *
     * @return whether the motor is at its target
     */
    public boolean isAtTarget() {
        return target != null && Math.abs(error(getPosition())) <= threshold;
    }

    /**
     * A command that drives at {@code speed} while it runs and stops when it ends. Made for
     * buttons: {@code controller.a().whileTrue(intake.driveCommand(0.8))}.
     *
     * @param speed from -1.0 to 1.0, negative for backwards
     * @return the command
     */
    public Command driveCommand(double speed) {
        return startEnd(() -> drive(speed), this::stop);
    }

    /**
     * A command that moves to {@code position} and finishes once it's there. The motor keeps
     * holding the position afterwards, so it can be chained:
     * {@code elevator.goToCommand(30).andThen(scoreCommand)}.
     *
     * @param position the position to move to
     * @return the command
     */
    public Command goToCommand(double position) {
        return runOnce(() -> goTo(position)).andThen(Commands.waitUntil(this::isAtTarget));
    }

    /**
     * A command that stops the motor and lets it coast for {@code seconds}, e.g. so a mechanism
     * can settle against a hard stop, then switches back to brake mode (even if interrupted).
     *
     * @param seconds how long to coast
     * @return the command
     */
    public Command brakelessReset(double seconds) {
        return startEnd(
            () -> {
                stop();
                setNeutralMode(NeutralModeValue.Coast);
            },
            () -> setNeutralMode(NeutralModeValue.Brake)
        ).withTimeout(seconds);
    }

    /**
     * Sets what the motor does when it has no power: brake to resist turning, or coast to spin
     * freely.
     *
     * @param neutralModeValue {@link NeutralModeValue#Brake} or {@link NeutralModeValue#Coast}
     */
    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        motor.setNeutralMode(neutralModeValue);
    }

    /**
     * Runs every loop (called by the command scheduler) and sets the motor's output from
     * whatever it was last told to do. You don't need to call this.
     */
    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            stop();
            // The robot cuts power itself when disabled, so ramp up from zero once re-enabled.
            if (accelerationLimiter != null) {
                accelerationLimiter.reset(0.0);
            }
        }

        double position = getPosition();
        double speed = target == null ? driveSpeed : positionSpeed(position);
        if ((position >= maxValue && speed > 0.0) || (position <= minValue && speed < 0.0)) {
            speed = 0.0;
        }

        output = accelerationLimiter != null ? accelerationLimiter.calculate(speed) : speed;
        motor.set(inverted ? -output : output);
    }

    /** Works out the speed to move toward the target at, or holdSpeed once there. */
    private double positionSpeed(double position) {
        double error = error(position);

        // Wider band to leave holding than to enter it, so it doesn't flicker at the edge.
        if (Math.abs(error) <= threshold) {
            holding = true;
        } else if (Math.abs(error) > threshold * 2.0) {
            holding = false;
        }
        if (holding) {
            return holdSpeed;
        }

        double maxSpeed = error > 0.0
            ? (motorUpSpeed != null ? motorUpSpeed : motorSpeed)
            : (motorDownSpeed != null ? motorDownSpeed : motorSpeed);
        return applyMinSpeed(MathUtil.clamp(kP * error, -maxSpeed, maxSpeed));
    }

    /** Distance from {@code position} to the target, going the short way around when continuous. */
    private double error(double position) {
        double error = target - position;
        return fullTurn > 0.0 ? MathUtil.inputModulus(error, -fullTurn / 2.0, fullTurn / 2.0) : error;
    }

    /** Raises a non-zero speed to at least minSpeed, keeping its direction. */
    private double applyMinSpeed(double speed) {
        return speed == 0.0 ? 0.0 : Math.copySign(Math.max(Math.abs(speed), minSpeed), speed);
    }

    /**
     * The motor's current position: from the position supplier if one is set, otherwise from the
     * TalonFX's built-in encoder in rotations (flipped when inverted).
     *
     * @return the current position
     */
    public double getPosition() {
        if (positionSupplier != null) {
            return positionSupplier.getAsDouble();
        }
        double position = motor.getPosition().getValueAsDouble();
        return inverted ? -position : position;
    }

    /**
     * The TalonFX this wraps, for anything this class doesn't cover, like current limits.
     *
     * @return the TalonFX
     */
    public TalonFX getMotor() {
        return motor;
    }

    /**
     * Same as {@link #getMotor}.
     *
     * @return the TalonFX
     */
    public TalonFX getTalonFX() {
        return motor;
    }

    /**
     * Whether the motor is inverted. See {@link #setInverted}.
     *
     * @return whether the motor is inverted
     */
    public boolean isInverted() {
        return inverted;
    }

    /** Throws if {@code value} isn't a number from {@code min} to {@code max}, otherwise returns it. */
    private static double check(double value, double min, double max, String name) {
        if (!(Double.isFinite(value) && value >= min && value <= max)) {
            throw new IllegalArgumentException(name + " must be between " + min + " and " + max + ", got " + value);
        }
        return value;
    }
}
