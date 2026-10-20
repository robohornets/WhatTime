package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls several {@link Motor}s together, e.g. the two motors on an elevator. Every method
 * does the same thing as the {@link Motor} method of the same name, on every motor in the group.
 *
 * <p>Each motor still uses its own position, so give inverted motors {@code inverted = true}
 * when creating them, or share one sensor with {@link #setPositionSupplier}.
 *
 * <pre>{@code
 * MotorGroup elevator = new MotorGroup(new Motor(11), new Motor(12, true)).setRange(0, 50);
 * controller.b().onTrue(elevator.goToCommand(30));
 * }</pre>
 */
public class MotorGroup extends SubsystemBase {
    private final List<Motor> motors;

    /**
     * Creates a group from some motors.
     *
     * @param motors the motors to control together; must not be empty
     */
    public MotorGroup(Motor... motors) {
        this(List.of(motors));
    }

    /**
     * Creates a group from a list of motors.
     *
     * @param motors the motors to control together; must not be empty
     */
    public MotorGroup(List<Motor> motors) {
        if (motors == null || motors.isEmpty()) {
            throw new IllegalArgumentException("motors cannot be empty.");
        }
        this.motors = List.copyOf(motors);
    }

    /** Runs {@code action} on every motor. */
    private MotorGroup each(Consumer<Motor> action) {
        motors.forEach(action);
        return this;
    }

    /**
     * Enables or disables every motor. See {@link Motor#setEnabled}.
     *
     * @param enabled whether the motors should be enabled
     * @return this group, for chaining
     */
    public MotorGroup setEnabled(boolean enabled) {
        return each(m -> m.setEnabled(enabled));
    }

    /**
     * Disables every motor if they're all enabled, otherwise enables every motor.
     *
     * @return whether the motors are now enabled
     */
    public boolean toggleEnabled() {
        setEnabled(!isEnabled());
        return isEnabled();
    }

    /**
     * Whether every motor is enabled.
     *
     * @return whether every motor is enabled
     */
    public boolean isEnabled() {
        return motors.stream().allMatch(Motor::isEnabled);
    }

    /**
     * Sets every motor to the same inversion. To invert only some motors, pass
     * {@code inverted = true} when creating them instead. See {@link Motor#setInverted}.
     *
     * @param inverted whether the motors should be inverted
     * @return this group, for chaining
     */
    public MotorGroup setInverted(boolean inverted) {
        return each(m -> m.setInverted(inverted));
    }

    /**
     * Sets every motor's lower soft limit. See {@link Motor#setRange}.
     *
     * @param minValue the lowest position the motors may drive to
     * @return this group, for chaining
     */
    public MotorGroup setMinValue(double minValue) {
        return each(m -> m.setMinValue(minValue));
    }

    /**
     * Sets every motor's upper soft limit. See {@link Motor#setRange}.
     *
     * @param maxValue the highest position the motors may drive to
     * @return this group, for chaining
     */
    public MotorGroup setMaxValue(double maxValue) {
        return each(m -> m.setMaxValue(maxValue));
    }

    /**
     * Sets every motor's soft limits. See {@link Motor#setRange}.
     *
     * @param minValue the lowest position the motors may drive to
     * @param maxValue the highest position the motors may drive to
     * @return this group, for chaining
     */
    public MotorGroup setRange(double minValue, double maxValue) {
        return each(m -> m.setRange(minValue, maxValue));
    }

    /**
     * Makes every motor take the shortest way around, for turrets. See {@link Motor#setContinuous}.
     *
     * @param min the start of one full turn, e.g. 0
     * @param max the end of one full turn, e.g. 1 for rotations or 360 for degrees
     * @return this group, for chaining
     */
    public MotorGroup setContinuous(double min, double max) {
        return each(m -> m.setContinuous(min, max));
    }

    /**
     * Sets every motor's smallest non-zero speed. See {@link Motor#setMinSpeed}.
     *
     * @param minSpeed from 0.0 to motorSpeed
     * @return this group, for chaining
     */
    public MotorGroup setMinSpeed(double minSpeed) {
        return each(m -> m.setMinSpeed(minSpeed));
    }

    /**
     * Sets every motor's drive speed and top {@code goTo} speed. See {@link Motor#setMotorSpeed}.
     *
     * @param motorSpeed from minSpeed to 1.0
     * @return this group, for chaining
     */
    public MotorGroup setMotorSpeed(double motorSpeed) {
        return each(m -> m.setMotorSpeed(motorSpeed));
    }

    /**
     * Sets every motor's top {@code goTo} speed toward higher positions. See {@link Motor#setMotorUpSpeed}.
     *
     * @param motorUpSpeed from minSpeed to 1.0, or {@code null} to use motorSpeed
     * @return this group, for chaining
     */
    public MotorGroup setMotorUpSpeed(Double motorUpSpeed) {
        return each(m -> m.setMotorUpSpeed(motorUpSpeed));
    }

    /**
     * Sets every motor's top {@code goTo} speed toward lower positions. See {@link Motor#setMotorDownSpeed}.
     *
     * @param motorDownSpeed from minSpeed to 1.0, or {@code null} to use motorSpeed
     * @return this group, for chaining
     */
    public MotorGroup setMotorDownSpeed(Double motorDownSpeed) {
        return each(m -> m.setMotorDownSpeed(motorDownSpeed));
    }

    /**
     * Makes every motor change speed gradually. See {@link Motor#setAccelerationTime}.
     *
     * @param seconds time to go from stopped to full speed; 0 turns acceleration off
     * @return this group, for chaining
     */
    public MotorGroup setAccelerationTime(double seconds) {
        return each(m -> m.setAccelerationTime(seconds));
    }

    /**
     * Sets every motor's output once at its target. See {@link Motor#setHoldSpeed}.
     *
     * @param holdSpeed from -1.0 to 1.0
     * @return this group, for chaining
     */
    public MotorGroup setHoldSpeed(double holdSpeed) {
        return each(m -> m.setHoldSpeed(holdSpeed));
    }

    /**
     * Sets how close to the target counts as being there, for every motor. See {@link Motor#setThreshold}.
     *
     * @param threshold the allowed distance from the target, in position units
     * @return this group, for chaining
     */
    public MotorGroup setThreshold(double threshold) {
        return each(m -> m.setThreshold(threshold));
    }

    /**
     * Sets every motor's proportional gain. See {@link Motor#setKP}.
     *
     * @param kP the proportional gain, 0 or more
     * @return this group, for chaining
     */
    public MotorGroup setKP(double kP) {
        return each(m -> m.setKP(kP));
    }

    /**
     * Makes every motor read its position from the same place, e.g. one shared encoder.
     * See {@link Motor#setPositionSupplier}.
     *
     * @param positionSupplier where to read the position from, or {@code null} for each motor's built-in encoder
     * @return this group, for chaining
     */
    public MotorGroup setPositionSupplier(DoubleSupplier positionSupplier) {
        return each(m -> m.setPositionSupplier(positionSupplier));
    }

    /**
     * Sets every motor's gear ratio. See {@link Motor#setGearRatio}.
     *
     * @param ratio motor rotations per mechanism rotation, from 0.001 to 1000
     * @return this group, for chaining
     */
    public MotorGroup setGearRatio(double ratio) {
        return each(m -> m.setGearRatio(ratio));
    }

    /**
     * Sets every motor's stator current limit. See {@link Motor#setStatorCurrentLimit}.
     *
     * @param amps the limit in amps, or {@code null} to turn it off
     * @return this group, for chaining
     */
    public MotorGroup setStatorCurrentLimit(Double amps) {
        return each(m -> m.setStatorCurrentLimit(amps));
    }

    /**
     * Sets every motor's supply current limit. See {@link Motor#setSupplyCurrentLimit}.
     *
     * @param amps the limit in amps, or {@code null} to turn it off
     * @return this group, for chaining
     */
    public MotorGroup setSupplyCurrentLimit(Double amps) {
        return each(m -> m.setSupplyCurrentLimit(amps));
    }

    /**
     * Sets every motor's lower supply current limit. See {@link Motor#setSupplyCurrentLowerLimit}.
     *
     * @param amps the lower limit in amps
     * @param seconds how long the supply limit must be hit first
     * @return this group, for chaining
     */
    public MotorGroup setSupplyCurrentLowerLimit(double amps, double seconds) {
        return each(m -> m.setSupplyCurrentLowerLimit(amps, seconds));
    }

    /**
     * Turns every motor's limit switches on or off. See {@link Motor#setLimitSwitchesEnabled}.
     *
     * @param enabled whether the TalonFXs should stop at their limit switches
     * @return this group, for chaining
     */
    public MotorGroup setLimitSwitchesEnabled(boolean enabled) {
        return each(m -> m.setLimitSwitchesEnabled(enabled));
    }

    /**
     * Makes every motor read its position from the same CANcoder. See {@link Motor#setCANcoder}.
     *
     * @param canId the CANcoder's CAN id
     * @param rotorToSensorRatio motor rotations per CANcoder rotation
     * @return this group, for chaining
     */
    public MotorGroup setCANcoder(int canId, double rotorToSensorRatio) {
        return each(m -> m.setCANcoder(canId, rotorToSensorRatio));
    }

    /**
     * Sets every motor's TalonFX PID gains. See {@link Motor#setPID}.
     *
     * @param kP volts per unit of error
     * @param kI volts per unit of error times seconds
     * @param kD volts per unit of error per second
     * @return this group, for chaining
     */
    public MotorGroup setPID(double kP, double kI, double kD) {
        return each(m -> m.setPID(kP, kI, kD));
    }

    /**
     * Sets every motor's TalonFX feedforward gains. See {@link Motor#setFeedforward}.
     *
     * @param kS volts to overcome friction
     * @param kV volts per rotation per second
     * @param kA volts per rotation per second squared
     * @return this group, for chaining
     */
    public MotorGroup setFeedforward(double kS, double kV, double kA) {
        return each(m -> m.setFeedforward(kS, kV, kA));
    }

    /**
     * Sets every motor's gravity feedforward. See {@link Motor#setGravity}. Each motor adds the
     * full kG, so split the voltage needed to hold the mechanism between them.
     *
     * @param kG volts each motor adds to hold the mechanism still
     * @param arm true for an arm, false for an elevator
     * @return this group, for chaining
     */
    public MotorGroup setGravity(double kG, boolean arm) {
        return each(m -> m.setGravity(kG, arm));
    }

    /**
     * Makes every motor's {@code goTo} run on its TalonFX with Motion Magic. See {@link Motor#setMotionMagic}.
     *
     * @param maxVelocity top speed in position units per second
     * @param maxAcceleration in position units per second squared
     * @return this group, for chaining
     */
    public MotorGroup setMotionMagic(double maxVelocity, double maxAcceleration) {
        return each(m -> m.setMotionMagic(maxVelocity, maxAcceleration));
    }

    /**
     * Sets how close to the velocity target counts as being there, for every motor.
     * See {@link Motor#setVelocityThreshold}.
     *
     * @param threshold the allowed difference, in rotations per second
     * @return this group, for chaining
     */
    public MotorGroup setVelocityThreshold(double threshold) {
        return each(m -> m.setVelocityThreshold(threshold));
    }

    /**
     * Publishes every motor to NetworkTables as {@code Motors/<name> 1}, {@code Motors/<name> 2}
     * and so on. See {@link Motor#setDashboardName}.
     *
     * @param name the name to publish under, or {@code null} to stop publishing
     * @return this group, for chaining
     */
    public MotorGroup setDashboardName(String name) {
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setDashboardName(name == null ? null : name + " " + (i + 1));
        }
        return this;
    }

    /**
     * Sets what every motor's simulation models. See {@link Motor#setSimulation}. Each motor is
     * simulated on its own, so give each one its share of the mechanism's moment of inertia.
     *
     * @param gearbox the motor, e.g. {@code DCMotor.getFalcon500(1)}
     * @param momentOfInertia each motor's share of the moment of inertia, in kg·m²
     * @return this group, for chaining
     */
    public MotorGroup setSimulation(DCMotor gearbox, double momentOfInertia) {
        return each(m -> m.setSimulation(gearbox, momentOfInertia));
    }

    /**
     * Makes every motor reset its position when its forward limit switch is pressed.
     * See {@link Motor#setForwardLimitSwitchPosition}.
     *
     * @param position the position to reset to, or {@code null} to not reset
     * @return this group, for chaining
     */
    public MotorGroup setForwardLimitSwitchPosition(Double position) {
        return each(m -> m.setForwardLimitSwitchPosition(position));
    }

    /**
     * Makes every motor reset its position when its reverse limit switch is pressed.
     * See {@link Motor#setReverseLimitSwitchPosition}.
     *
     * @param position the position to reset to, or {@code null} to not reset
     * @return this group, for chaining
     */
    public MotorGroup setReverseLimitSwitchPosition(Double position) {
        return each(m -> m.setReverseLimitSwitchPosition(position));
    }

    /**
     * Changes any other TalonFX setting on every motor. See {@link Motor#configure}.
     *
     * @param changes edits to make to each motor's configuration
     * @return this group, for chaining
     */
    public MotorGroup configure(Consumer<TalonFXConfiguration> changes) {
        return each(m -> m.configure(changes));
    }

    /** Drives every motor forward at its motorSpeed until stopped. See {@link Motor#drive()}. */
    public void drive() {
        each(Motor::drive);
    }

    /**
     * Drives every motor at its motorSpeed until stopped. See {@link Motor#drive(boolean)}.
     *
     * @param reverse whether to drive backwards
     */
    public void drive(boolean reverse) {
        each(m -> m.drive(reverse));
    }

    /**
     * Drives every motor at {@code speed} until stopped. See {@link Motor#drive(double)}.
     *
     * @param speed from -1.0 to 1.0, negative for backwards
     */
    public void drive(double speed) {
        each(m -> m.drive(speed));
    }

    /**
     * Moves every motor to {@code position} and holds it there. See {@link Motor#goTo}.
     *
     * @param position the position to move to
     */
    public void goTo(double position) {
        each(m -> m.goTo(position));
    }

    /**
     * Same as {@link #goTo}.
     *
     * @param position the position to move to
     */
    public void setTarget(double position) {
        goTo(position);
    }

    /**
     * Spins every motor at {@code velocity}. See {@link Motor#setVelocity}.
     *
     * @param velocity rotations per second, negative for backwards
     */
    public void setVelocity(double velocity) {
        each(m -> m.setVelocity(velocity));
    }

    /**
     * Whether every motor is at its target velocity. See {@link Motor#isAtVelocity}.
     *
     * @return whether every motor is at its target velocity
     */
    public boolean isAtVelocity() {
        return motors.stream().allMatch(Motor::isAtVelocity);
    }

    /**
     * Tells every motor's built-in encoder it's now at {@code position}. See {@link Motor#resetPosition}.
     *
     * @param position the position the motors are at right now
     */
    public void resetPosition(double position) {
        each(m -> m.resetPosition(position));
    }

    /** Stops every motor and clears their targets. See {@link Motor#stop}. */
    public void stop() {
        each(Motor::stop);
    }

    /**
     * Whether every motor is at its target. See {@link Motor#isAtTarget}.
     *
     * @return whether every motor is at its target
     */
    public boolean isAtTarget() {
        return motors.stream().allMatch(Motor::isAtTarget);
    }

    /**
     * Sets every motor's neutral mode. See {@link Motor#setNeutralMode}.
     *
     * @param neutralModeValue {@link NeutralModeValue#Brake} or {@link NeutralModeValue#Coast}
     * @return this group, for chaining
     */
    public MotorGroup setNeutralMode(NeutralModeValue neutralModeValue) {
        return each(m -> m.setNeutralMode(neutralModeValue));
    }

    /**
     * A command that drives every motor at {@code speed} while it runs and stops them when it
     * ends. See {@link Motor#driveCommand}.
     *
     * @param speed from -1.0 to 1.0, negative for backwards
     * @return the command
     */
    public Command driveCommand(double speed) {
        return startEnd(() -> drive(speed), this::stop);
    }

    /**
     * A command that moves every motor to {@code position} and finishes once they're all there.
     * See {@link Motor#goToCommand}.
     *
     * @param position the position to move to
     * @return the command
     */
    public Command goToCommand(double position) {
        return runOnce(() -> goTo(position)).andThen(Commands.waitUntil(this::isAtTarget));
    }

    /**
     * A command that spins every motor at {@code velocity} while it runs and stops them when it
     * ends. See {@link Motor#velocityCommand}.
     *
     * @param velocity rotations per second, negative for backwards
     * @return the command
     */
    public Command velocityCommand(double velocity) {
        return startEnd(() -> setVelocity(velocity), this::stop);
    }

    /**
     * A command that drives every motor at {@code speed} until any of them stalls or reaches a
     * limit switch, then stops them all and resets every position to {@code position}.
     * See {@link Motor#homeCommand}.
     *
     * @param speed from -1.0 to 1.0; keep it slow
     * @param stallCurrent the stator current in amps that means a motor has hit the stop
     * @param position the position of the hard stop
     * @return the command
     */
    public Command homeCommand(double speed, double stallCurrent, double position) {
        return runOnce(() -> {
            drive(speed);
            each(m -> m.setHoming(true));
        })
            .andThen(Commands.waitSeconds(0.25))
            .andThen(Commands.waitUntil(() -> motors.stream().anyMatch(m -> m.isStalledOrAtSwitch(speed, stallCurrent))))
            .finallyDo(interrupted -> {
                each(m -> m.setHoming(false));
                stop();
                if (!interrupted) {
                    resetPosition(position);
                }
            });
    }

    /**
     * A command that stops every motor and lets them coast for {@code seconds}, then switches
     * back to brake mode. See {@link Motor#brakelessReset}.
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
     * The motors in this group.
     *
     * @return the motors, in the order they were given
     */
    public List<Motor> getMotors() {
        return motors;
    }
}
