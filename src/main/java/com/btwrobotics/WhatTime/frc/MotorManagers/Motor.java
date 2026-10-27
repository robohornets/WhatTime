package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A TalonFX that drives at a speed, spins at a velocity, or moves to and holds a position.
 *
 * <p>The motor updates itself every loop, so one call is enough: {@code drive(0.5)} keeps
 * spinning until {@link #stop()}, {@code setVelocity(80)} holds 80 rotations per second, and
 * {@code goTo(30)} moves there and holds. The motor stops whenever the robot is disabled.
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
    /** The TalonFX's settings, read from it on creation. Kept here so applying one group doesn't undo another. */
    private final TalonFXConfiguration config = new TalonFXConfiguration();

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
    /** goTo's PID gains, in volts like the TalonFX's. 1.2 V is the old default of 0.1 output per unit. */
    private double kP = 1.2;
    private double kI = 0.0;
    private double kD = 0.0;
    /** Running integral and last error for goTo's PID, or NaN before the first loop of a move. */
    private double integral;
    private double lastError = Double.NaN;
    /** FPGA time of the last goTo loop, for measuring how long each loop really took. */
    private double lastTime;
    /** FPGA time this motor's settings were last re-applied by {@link #enforceConfig}. */
    private double lastConfigApply = Double.NEGATIVE_INFINITY;
    /** Limits how fast the output can change, or null when acceleration is off. */
    private SlewRateLimiter accelerationLimiter;
    private DoubleSupplier positionSupplier;
    /** Whether goTo uses the TalonFX's Motion Magic instead of the kP loop here. */
    private boolean motionMagic;
    private double velocityThreshold = 1.0;
    /** NetworkTables table to publish to, or null to not publish. */
    private String dashboardTable;
    private DCMotor simGearbox = DCMotor.getKrakenX60(1);
    private double simMomentOfInertia = 0.01;
    /** Created on the first simulation loop, so it uses the final gear ratio. */
    private DCMotorSim sim;

    /** Speed to drive at when there is no position target. */
    private double driveSpeed;
    /** Position to move to and hold, or null when driving. */
    private Double target;
    /** Velocity for the TalonFX to hold, or null when not in velocity mode. */
    private Double targetVelocity;
    private boolean holding;
    /** True while homing, so the soft limits don't stop it. */
    private boolean homing;
    /** Output sent to the motor last loop. */
    private double output;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    /** Output is sent as a share of 12 V, not of the battery, so it doesn't weaken as the battery sags. */
    private final VoltageOut voltageRequest = new VoltageOut(0);

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
        // Start from what's on the TalonFX, so settings made in Phoenix Tuner aren't undone.
        motor.getConfigurator().refresh(config);
        setInverted(inverted);
    }

    /**
     * Creates a motor from an existing TalonFX object. Settings already on the TalonFX are kept,
     * apart from inversion, which is set to {@code inverted}.
     *
     * @param motor the TalonFX motor
     * @param inverted whether the motor should be inverted
     * @return A Motor object that wraps the input TalonFX object
     */
    public static Motor of(TalonFX motor, boolean inverted) {
        return new Motor(motor, inverted);
    }

    /**
     * Flips the motor's direction, so positive speeds spin it the other way. This is the TalonFX's
     * own inversion setting, so its built-in encoder and limit switches flip to match: positions
     * still go up when driving forward, and the forward limit switch still stops forward driving.
     *
     * @param inverted whether the motor should be inverted
     * @return this motor, for chaining
     */
    public Motor setInverted(boolean inverted) {
        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config.MotorOutput);
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
     * Works no matter how many times the mechanism has spun around.
     *
     * <p>For a turret that can only spin so far before its wires run out, add soft limits:
     * {@code setContinuous(0, 1).setRange(-2, 3)} allows 2 turns back and 3 turns forward from 0.
     * {@link #goTo} then goes to whichever matching angle inside the limits is closest, taking
     * the long way around when the short way would cross a limit. This needs a position that
     * keeps counting past a full turn (like the built-in encoder), not one that wraps back to 0.
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
     * Sets only goTo's proportional gain, as output (-1 to 1) per unit of error. Same as
     * {@code setPID(kP * 12, kI, kD)} without touching kI and kD.
     *
     * @param kP the proportional gain, 0 or more
     * @return this motor, for chaining
     * @deprecated use {@link #setPID}, which sets kP, kI and kD in volts
     */
    @Deprecated
    public Motor setKP(double kP) {
        return setPID(check(kP, 0.0, Double.POSITIVE_INFINITY, "kP") * 12.0, kI, kD);
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
     * Sets how many motor rotations make one rotation of the mechanism, e.g. 10 for a 10:1
     * gearbox. The built-in encoder then reads in mechanism rotations, so positions, soft limits,
     * targets, threshold and kP all use those units. Has no effect on a position supplier.
     *
     * @param ratio motor rotations per mechanism rotation, from 0.001 to 1000; defaults to 1
     * @return this motor, for chaining
     */
    public Motor setGearRatio(double ratio) {
        config.Feedback.SensorToMechanismRatio = check(ratio, 0.001, 1000.0, "gearRatio");
        motor.getConfigurator().apply(config.Feedback);
        return this;
    }

    /**
     * Limits the current through the motor itself, which limits how hard it can push. Lower it
     * so a mechanism doesn't burn itself out pushing against something, e.g. an intake roller
     * jammed on a game piece.
     *
     * @param amps the limit in amps (0 to 800), or {@code null} to turn it off; the TalonFX
     *     default is 120
     * @return this motor, for chaining
     */
    public Motor setStatorCurrentLimit(Double amps) {
        if (amps != null) {
            config.CurrentLimits.StatorCurrentLimit = check(amps, 0.0, 800.0, "statorCurrentLimit");
        }
        config.CurrentLimits.StatorCurrentLimitEnable = amps != null;
        motor.getConfigurator().apply(config.CurrentLimits);
        return this;
    }

    /**
     * Limits the current the motor draws from the battery, so it doesn't trip breakers or brown
     * out the robot. After the limit has been hit for SupplyCurrentLowerTime (1 second by
     * default), the TalonFX drops to SupplyCurrentLowerLimit (40 A by default); change those with
     * {@link #configure}.
     *
     * @param amps the limit in amps (0 to 800), or {@code null} to turn it off; the TalonFX
     *     default is 70
     * @return this motor, for chaining
     */
    public Motor setSupplyCurrentLimit(Double amps) {
        if (amps != null) {
            config.CurrentLimits.SupplyCurrentLimit = check(amps, 0.0, 800.0, "supplyCurrentLimit");
        }
        config.CurrentLimits.SupplyCurrentLimitEnable = amps != null;
        motor.getConfigurator().apply(config.CurrentLimits);
        return this;
    }

    /**
     * Sets the lower supply current limit the TalonFX drops to once the supply limit has been hit
     * for {@code seconds}, so a motor stalled for a long time doesn't drain the battery.
     *
     * @param amps the lower limit in amps (0 to 500); the TalonFX default is 40
     * @param seconds how long the supply limit must be hit first (0 to 2.5); the TalonFX default is 1
     * @return this motor, for chaining
     */
    public Motor setSupplyCurrentLowerLimit(double amps, double seconds) {
        config.CurrentLimits.SupplyCurrentLowerLimit = check(amps, 0.0, 500.0, "supplyCurrentLowerLimit");
        config.CurrentLimits.SupplyCurrentLowerTime = check(seconds, 0.0, 2.5, "supplyCurrentLowerTime");
        motor.getConfigurator().apply(config.CurrentLimits);
        return this;
    }

    /**
     * Turns both limit switches on or off. While on, the TalonFX stops the motor whenever a limit
     * switch wired to it is pressed. They're on by default.
     *
     * @param enabled whether the TalonFX should stop at its limit switches
     * @return this motor, for chaining
     */
    public Motor setLimitSwitchesEnabled(boolean enabled) {
        config.HardwareLimitSwitch.ForwardLimitEnable = enabled;
        config.HardwareLimitSwitch.ReverseLimitEnable = enabled;
        motor.getConfigurator().apply(config.HardwareLimitSwitch);
        return this;
    }

    /**
     * Makes the TalonFX read its position from a CANcoder on the same CAN bus instead of its
     * built-in encoder, e.g. an absolute encoder on an arm so it knows where it is at startup. Set
     * the CANcoder's zero in Phoenix Tuner. After this, {@link #setGearRatio} is CANcoder
     * rotations per mechanism rotation, which is 1 when the CANcoder is on the mechanism itself.
     *
     * @param canId the CANcoder's CAN id
     * @param rotorToSensorRatio motor rotations per CANcoder rotation, e.g. 50 for a 50:1 gearbox
     *     between the motor and the CANcoder
     * @return this motor, for chaining
     */
    public Motor setCANcoder(int canId, double rotorToSensorRatio) {
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = canId;
        config.Feedback.RotorToSensorRatio = check(rotorToSensorRatio, 0.001, 1000.0, "rotorToSensorRatio");
        motor.getConfigurator().apply(config.Feedback);
        return this;
    }

    /**
     * Sets the PID gains used by {@link #goTo} (both the default loop and Motion Magic) and by
     * {@link #setVelocity}. These are in volts: kP is volts per position unit of error (or per
     * rotation per second for velocity), so 12 means full output at 1 unit away. The default goTo
     * starts at kP 1.2, kI 0, kD 0; the TalonFX's own gains stay 0 until this is called.
     *
     * @param kP volts per unit of error
     * @param kI volts per unit of error times seconds
     * @param kD volts per unit of error per second
     * @return this motor, for chaining
     */
    public Motor setPID(double kP, double kI, double kD) {
        this.kP = config.Slot0.kP = check(kP, 0.0, Double.POSITIVE_INFINITY, "kP");
        this.kI = config.Slot0.kI = check(kI, 0.0, Double.POSITIVE_INFINITY, "kI");
        this.kD = config.Slot0.kD = check(kD, 0.0, Double.POSITIVE_INFINITY, "kD");
        motor.getConfigurator().apply(config.Slot0);
        return this;
    }

    /**
     * Sets the feedforward gains, used along with {@link #setPID}. kS also applies to the default
     * goTo; kV and kA only to Motion Magic and {@link #setVelocity}. For velocity, kV does most of
     * the work: start at 12 divided by the mechanism's top speed in rotations per second. All
     * three default to 0.
     *
     * @param kS volts to overcome friction
     * @param kV volts per rotation per second
     * @param kA volts per rotation per second squared
     * @return this motor, for chaining
     */
    public Motor setFeedforward(double kS, double kV, double kA) {
        config.Slot0.kS = check(kS, 0.0, Double.POSITIVE_INFINITY, "kS");
        config.Slot0.kV = check(kV, 0.0, Double.POSITIVE_INFINITY, "kV");
        config.Slot0.kA = check(kA, 0.0, Double.POSITIVE_INFINITY, "kA");
        motor.getConfigurator().apply(config.Slot0);
        return this;
    }

    /**
     * Sets the volts added to hold a mechanism up against gravity, used along with
     * {@link #setPID} in every goTo mode. For an arm, the push is scaled by the arm's angle, so position 0 must be
     * the arm sticking straight out sideways and positions must be in rotations of the arm (use
     * {@link #setGearRatio}). Defaults to 0.
     *
     * @param kG volts to hold the mechanism still
     * @param arm true for an arm, false for an elevator or anything else gravity pulls on equally
     * @return this motor, for chaining
     */
    public Motor setGravity(double kG, boolean arm) {
        config.Slot0.kG = check(kG, -12.0, 12.0, "kG");
        config.Slot0.GravityType = arm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
        motor.getConfigurator().apply(config.Slot0);
        return this;
    }

    /**
     * Makes {@link #goTo} run on the TalonFX itself with Motion Magic: it plans a smooth move
     * that speeds up, cruises and slows down, and follows it with {@link #setPID},
     * {@link #setFeedforward} and {@link #setGravity}, 1000 times a second. This is smoother and
     * more accurate than the default goTo, but needs those gains tuned. kP, motorSpeed,
     * motorUpSpeed, motorDownSpeed, minSpeed, holdSpeed and accelerationTime don't apply to it.
     * It uses the TalonFX's own sensor (built-in or {@link #setCANcoder}), not a position supplier.
     *
     * @param maxVelocity top speed in position units per second
     * @param maxAcceleration how fast to speed up and slow down, in position units per second squared
     * @return this motor, for chaining
     */
    public Motor setMotionMagic(double maxVelocity, double maxAcceleration) {
        config.MotionMagic.MotionMagicCruiseVelocity = check(maxVelocity, Double.MIN_VALUE, Double.POSITIVE_INFINITY, "maxVelocity");
        config.MotionMagic.MotionMagicAcceleration = check(maxAcceleration, Double.MIN_VALUE, Double.POSITIVE_INFINITY, "maxAcceleration");
        motor.getConfigurator().apply(config.MotionMagic);
        motionMagic = true;
        return this;
    }

    /**
     * Sets how close to the {@link #setVelocity} target counts as being there.
     *
     * @param threshold the allowed difference, in rotations per second; defaults to 1
     * @return this motor, for chaining
     */
    public Motor setVelocityThreshold(double threshold) {
        this.velocityThreshold = check(threshold, 0.0, Double.POSITIVE_INFINITY, "velocityThreshold");
        return this;
    }

    /**
     * Publishes this motor's position, target, velocity, output, currents and temperature to
     * NetworkTables every loop, under {@code Motors/<name>}.
     *
     * @param name the name to publish under, or {@code null} to stop publishing
     * @return this motor, for chaining
     */
    public Motor setDashboardName(String name) {
        this.dashboardTable = name == null ? null : "Motors/" + name;
        return this;
    }

    /**
     * Sets what the simulation models: which motor it is and how heavy the mechanism is. Only used
     * when running the robot simulator. Defaults to a Kraken X60 turning 0.01 kg·m².
     *
     * @param gearbox the motor, e.g. {@code DCMotor.getFalcon500(1)}
     * @param momentOfInertia the mechanism's moment of inertia in kg·m², more for heavier mechanisms
     * @return this motor, for chaining
     */
    public Motor setSimulation(DCMotor gearbox, double momentOfInertia) {
        this.simGearbox = gearbox;
        this.simMomentOfInertia = check(momentOfInertia, Double.MIN_VALUE, Double.POSITIVE_INFINITY, "momentOfInertia");
        sim = null;
        return this;
    }

    /**
     * Resets the built-in encoder to {@code position} whenever the forward limit switch is
     * pressed, so the mechanism re-zeroes itself every time it reaches the switch. The TalonFX
     * already stops the motor at a pressed limit switch by default.
     *
     * @param position the position to reset to, or {@code null} (the default) to not reset
     * @return this motor, for chaining
     */
    public Motor setForwardLimitSwitchPosition(Double position) {
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = position != null;
        if (position != null) {
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = position;
        }
        motor.getConfigurator().apply(config.HardwareLimitSwitch);
        return this;
    }

    /**
     * Same as {@link #setForwardLimitSwitchPosition}, for the reverse limit switch, e.g.
     * {@code setReverseLimitSwitchPosition(0.0)} for a switch at the bottom of an elevator.
     *
     * @param position the position to reset to, or {@code null} (the default) to not reset
     * @return this motor, for chaining
     */
    public Motor setReverseLimitSwitchPosition(Double position) {
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = position != null;
        if (position != null) {
            config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = position;
        }
        motor.getConfigurator().apply(config.HardwareLimitSwitch);
        return this;
    }

    /**
     * Changes any other TalonFX setting, e.g. {@code configure(c -> c.Audio.BeepOnBoot = false)}.
     * The configuration starts as whatever was on the TalonFX when this motor was created (the
     * TalonFX defaults, unless changed in Phoenix Tuner) plus whatever this motor's other setters
     * changed. Motor's own features (soft limits, acceleration, speeds) work on top of these.
     *
     * @param changes edits to make to the configuration
     * @return this motor, for chaining
     */
    public Motor configure(Consumer<TalonFXConfiguration> changes) {
        changes.accept(config);
        motor.getConfigurator().apply(config);
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
        targetVelocity = null;
        driveSpeed = applyMinSpeed(MathUtil.clamp(speed, -1.0, 1.0));
    }

    /**
     * Moves to {@code position} and holds it there until stopped or told to do something else.
     * The position is clamped into the soft limits. When continuous, it's an angle, and the
     * closest matching angle (inside the soft limits, if any) is used (see {@link #setContinuous}).
     *
     * @param position the position to move to, in the same units as {@link #getPosition}
     */
    public void goTo(double position) {
        if (!enabled) {
            return;
        }
        if (fullTurn > 0.0) {
            position = nearestInRange(position, getPosition(), fullTurn, minValue, maxValue);
        }
        target = MathUtil.clamp(position, minValue, maxValue);
        targetVelocity = null;
        holding = false;
        integral = 0.0;
        lastError = Double.NaN;
    }

    /**
     * Spins at {@code velocity} until stopped or told to do something else, with the TalonFX
     * holding that speed under load, e.g. for a shooter. Needs {@link #setFeedforward} (mainly
     * kV) and usually a little kP from {@link #setPID}. The soft limits still apply.
     *
     * @param velocity rotations per second (mechanism rotations with {@link #setGearRatio}),
     *     negative for backwards
     */
    public void setVelocity(double velocity) {
        if (!enabled) {
            return;
        }
        target = null;
        driveSpeed = 0.0;
        targetVelocity = velocity;
    }

    /**
     * Whether the motor has a {@link #setVelocity} target and is within the velocity threshold of
     * it, e.g. to only feed a shooter once it's up to speed.
     *
     * @return whether the motor is at its target velocity
     */
    public boolean isAtVelocity() {
        return targetVelocity != null && Math.abs(targetVelocity - getVelocity()) <= velocityThreshold;
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
        targetVelocity = null;
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
     * A command that spins at {@code velocity} while it runs and stops when it ends:
     * {@code controller.rightTrigger().whileTrue(shooter.velocityCommand(80))}. See
     * {@link #setVelocity}.
     *
     * @param velocity rotations per second, negative for backwards
     * @return the command
     */
    public Command velocityCommand(double velocity) {
        return startEnd(() -> setVelocity(velocity), this::stop);
    }

    /**
     * A command that finds a mechanism's zero without a limit switch: it drives slowly at
     * {@code speed} until the motor stalls against a hard stop (its stator current reaches
     * {@code stallCurrent}) or a limit switch in that direction is pressed, then stops and resets
     * the position to {@code position}. The soft limits are ignored while it runs. If interrupted,
     * it stops without resetting.
     *
     * <pre>{@code
     * elevator.homeCommand(-0.15, 20, 0); // slowly down until it hits bottom, then call that 0
     * }</pre>
     *
     * @param speed from -1.0 to 1.0; keep it slow so the mechanism isn't damaged
     * @param stallCurrent the stator current in amps that means it has hit the stop
     * @param position the position of the hard stop
     * @return the command
     */
    public Command homeCommand(double speed, double stallCurrent, double position) {
        return runOnce(() -> {
            drive(speed);
            homing = true;
        })
            // Starting up draws a burst of current, so give it a moment before watching for a stall.
            .andThen(Commands.waitSeconds(0.25))
            .andThen(Commands.waitUntil(() -> isStalledOrAtSwitch(speed, stallCurrent)))
            .finallyDo(interrupted -> {
                homing = false;
                stop();
                if (!interrupted) {
                    resetPosition(position);
                }
            });
    }

    /** Whether homing in the direction of {@code speed} has reached the end. Used by MotorGroup too. */
    boolean isStalledOrAtSwitch(double speed, double stallCurrent) {
        return Math.abs(getStatorCurrent()) >= stallCurrent
            || (speed > 0.0 ? isForwardLimitSwitchPressed() : isReverseLimitSwitchPressed());
    }

    /** Lets MotorGroup's homeCommand ignore the soft limits. */
    void setHoming(boolean homing) {
        this.homing = homing;
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
     * @param neutralModeValue {@link NeutralModeValue#Brake} or {@link NeutralModeValue#Coast};
     *     the TalonFX default is Coast
     * @return this motor, for chaining
     */
    public Motor setNeutralMode(NeutralModeValue neutralModeValue) {
        config.MotorOutput.NeutralMode = neutralModeValue;
        motor.setNeutralMode(neutralModeValue);
        return this;
    }

    /**
     * Runs every loop (called by the command scheduler) and sets the motor's output from
     * whatever it was last told to do. You don't need to call this.
     */
    @Override
    public void periodic() {
        enforceConfig();
        if (DriverStation.isDisabled()) {
            stop();
            // The robot cuts power itself when disabled, so ramp up from zero once re-enabled.
            if (accelerationLimiter != null) {
                accelerationLimiter.reset(0.0);
            }
        }

        double position = getPosition();
        if (targetVelocity != null && !pastLimit(position, targetVelocity)) {
            runOnTalonFX(velocityRequest.withVelocity(targetVelocity));
        } else if (target != null && motionMagic) {
            runOnTalonFX(motionMagicRequest.withPosition(target));
        } else {
            double speed = target == null ? driveSpeed : positionSpeed(position);
            if (pastLimit(position, speed)) {
                speed = 0.0;
            }
            output = accelerationLimiter != null ? accelerationLimiter.calculate(speed) : speed;
            motor.setControl(voltageRequest.withOutput(output * 12.0));
        }

        if (dashboardTable != null) {
            publishToDashboard(position);
        }
    }

    /**
     * Puts this motor's settings back if something else changed them: the TalonFX rebooting, or
     * code applying its own configuration to the TalonFX. Only this class should configure it,
     * through its setters and {@link #configure}. Inversion is checked every loop since it's the
     * setting most often clobbered; retries are at most once a second so a missing motor doesn't
     * stall the loop.
     */
    private void enforceConfig() {
        AppliedRotorPolarityValue expected = isInverted()
            ? AppliedRotorPolarityValue.PositiveIsClockwise
            : AppliedRotorPolarityValue.PositiveIsCounterClockwise;
        var polarity = motor.getAppliedRotorPolarity();
        boolean changed = motor.hasResetOccurred()
            || (polarity.getStatus().isOK() && polarity.getValue() != expected);
        double now = Timer.getFPGATimestamp();
        if (changed && now - lastConfigApply >= 1.0) {
            lastConfigApply = now;
            motor.getConfigurator().apply(config);
        }
    }

    /** Sends a closed-loop request, and keeps the acceleration limit in step so switching back to driving is smooth. */
    private void runOnTalonFX(ControlRequest request) {
        motor.setControl(request);
        output = motor.getMotorVoltage().getValueAsDouble() / 12.0;
        if (accelerationLimiter != null) {
            accelerationLimiter.reset(output);
        }
    }

    /** Whether moving in the direction of {@code speed} would go further past a soft limit. */
    private boolean pastLimit(double position, double speed) {
        return !homing && ((position >= maxValue && speed > 0.0) || (position <= minValue && speed < 0.0));
    }

    private void publishToDashboard(double position) {
        NetworkTablesUtil.put(dashboardTable, "Position", position);
        NetworkTablesUtil.put(dashboardTable, "Target", target != null ? target : Double.NaN);
        NetworkTablesUtil.put(dashboardTable, "At Target", isAtTarget());
        NetworkTablesUtil.put(dashboardTable, "Velocity", getVelocity());
        NetworkTablesUtil.put(dashboardTable, "Target Velocity", targetVelocity != null ? targetVelocity : Double.NaN);
        NetworkTablesUtil.put(dashboardTable, "Output", output);
        NetworkTablesUtil.put(dashboardTable, "Stator Current", getStatorCurrent());
        NetworkTablesUtil.put(dashboardTable, "Supply Current", getSupplyCurrent());
        NetworkTablesUtil.put(dashboardTable, "Temperature", getTemperature());
        NetworkTablesUtil.put(dashboardTable, "Enabled", enabled);
    }

    /**
     * Runs every loop in the robot simulator and feeds the TalonFX a simulated motor, so
     * {@link #goTo}, soft limits, velocity and the dashboard all work without a robot. You don't
     * need to call this. See {@link #setSimulation}. Only the built-in encoder is simulated, not a
     * CANcoder or position supplier.
     */
    @Override
    public void simulationPeriodic() {
        double ratio = config.Feedback.SensorToMechanismRatio;
        if (sim == null) {
            // ponytail: free-spinning mechanism, no gravity or hard stops; model those yourself if tuning depends on them.
            sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(simGearbox, simMomentOfInertia, ratio), simGearbox);
        }

        TalonFXSimState simState = motor.getSimState();
        simState.Orientation = isInverted() ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        sim.setInputVoltage(simState.getMotorVoltage());
        sim.update(0.020);
        simState.setRawRotorPosition(sim.getAngularPositionRotations() * ratio);
        simState.setRotorVelocity(Units.radiansToRotations(sim.getAngularVelocityRadPerSec()) * ratio);
    }

    /**
     * Works out the speed to move toward the target at, or holdSpeed once there. Uses the PID
     * gains plus kS and kG from {@link #setFeedforward} and {@link #setGravity}; kV and kA need a
     * planned speed, so they only apply with Motion Magic or velocity.
     */
    private double positionSpeed(double position) {
        double error = error(position);
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        // Wider band to leave holding than to enter it, so it doesn't flicker at the edge.
        if (Math.abs(error) <= threshold) {
            holding = true;
        } else if (Math.abs(error) > threshold * 2.0) {
            holding = false;
        }
        if (holding) {
            // Keep the integral (frozen) and gravity push so it doesn't sag out of the band.
            lastError = Double.NaN;
            return MathUtil.clamp(holdSpeed + (kI * integral + gravityVolts(position)) / 12.0, -1.0, 1.0);
        }

        // First loop of a move, or a long gap (e.g. after being disabled): skip I and D this loop.
        boolean fresh = Double.isNaN(lastError) || dt <= 0.0 || dt > 0.1;
        if (fresh) {
            dt = 0.0;
        }
        double derivative = fresh ? 0.0 : (error - lastError) / dt;
        lastError = error;
        integral += error * dt;
        double volts = kP * error + kI * integral + kD * derivative
            + config.Slot0.kS * Math.signum(error) + gravityVolts(position);

        double maxSpeed = error > 0.0
            ? (motorUpSpeed != null ? motorUpSpeed : motorSpeed)
            : (motorDownSpeed != null ? motorDownSpeed : motorSpeed);
        double speed = volts / 12.0;
        // Stop the integral winding up while the output is capped.
        if (Math.abs(speed) > maxSpeed) {
            integral -= error * dt;
        }
        return applyMinSpeed(MathUtil.clamp(speed, -maxSpeed, maxSpeed));
    }

    /** Volts to hold against gravity from {@link #setGravity}, scaled by angle for an arm. */
    private double gravityVolts(double position) {
        return config.Slot0.GravityType == GravityTypeValue.Arm_Cosine
            ? config.Slot0.kG * Math.cos(Units.rotationsToRadians(position))
            : config.Slot0.kG;
    }

    /**
     * Distance from {@code position} to the target, going the short way around when continuous.
     * With soft limits, goTo already picked which way to go, so the distance is used as-is.
     */
    private double error(double position) {
        double error = target - position;
        return fullTurn > 0.0 && !hasRange() ? MathUtil.inputModulus(error, -fullTurn / 2.0, fullTurn / 2.0) : error;
    }

    /** Whether either soft limit is set. */
    private boolean hasRange() {
        return minValue != Double.NEGATIVE_INFINITY || maxValue != Double.POSITIVE_INFINITY;
    }

    /**
     * Of every position matching {@code angle} (angle plus or minus whole turns), the one inside
     * min..max closest to {@code current}. If none fit, one just outside, which goTo then clamps.
     */
    static double nearestInRange(double angle, double current, double fullTurn, double min, double max) {
        double best = current + MathUtil.inputModulus(angle - current, -fullTurn / 2.0, fullTurn / 2.0);
        if (best > max) {
            best -= Math.ceil((best - max) / fullTurn) * fullTurn;
        }
        if (best < min) {
            best += Math.ceil((min - best) / fullTurn) * fullTurn;
        }
        return best;
    }

    /** Raises a non-zero speed to at least minSpeed, keeping its direction. */
    private double applyMinSpeed(double speed) {
        return speed == 0.0 ? 0.0 : Math.copySign(Math.max(Math.abs(speed), minSpeed), speed);
    }

    /**
     * The motor's current position: from the position supplier if one is set, otherwise from the
     * TalonFX's built-in encoder in rotations (mechanism rotations with {@link #setGearRatio}).
     *
     * @return the current position
     */
    public double getPosition() {
        if (positionSupplier != null) {
            return positionSupplier.getAsDouble();
        }
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Tells the built-in encoder it's now at {@code position}, e.g. {@code resetPosition(0)} once
     * an elevator is at the bottom. This doesn't move the motor, and has no effect on a position
     * supplier.
     *
     * @param position the position the motor is at right now
     */
    public void resetPosition(double position) {
        motor.setPosition(position);
    }

    /**
     * How fast the motor is spinning, from the TalonFX's sensor (not a position supplier).
     *
     * @return rotations per second (mechanism rotations with {@link #setGearRatio})
     */
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * The current through the motor itself, which shows how hard it's pushing.
     *
     * @return amps; negative while pushing backwards
     */
    public double getStatorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * The current the motor is drawing from the battery.
     *
     * @return amps
     */
    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * The TalonFX's temperature. It starts limiting itself around 100 °C.
     *
     * @return degrees Celsius
     */
    public double getTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * Whether the forward limit switch wired to the TalonFX is pressed, taking normally-closed
     * switches (set with {@link #configure}) into account.
     *
     * @return whether the forward limit switch is pressed
     */
    public boolean isForwardLimitSwitchPressed() {
        boolean closed = motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
        return closed == (config.HardwareLimitSwitch.ForwardLimitType == ForwardLimitTypeValue.NormallyOpen);
    }

    /**
     * Whether the reverse limit switch wired to the TalonFX is pressed. See
     * {@link #isForwardLimitSwitchPressed}.
     *
     * @return whether the reverse limit switch is pressed
     */
    public boolean isReverseLimitSwitchPressed() {
        boolean closed = motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
        return closed == (config.HardwareLimitSwitch.ReverseLimitType == ReverseLimitTypeValue.NormallyOpen);
    }

    /**
     * The TalonFX this wraps, for anything this class doesn't cover.
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
        return config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
    }

    /** Throws if {@code value} isn't a number from {@code min} to {@code max}, otherwise returns it. */
    private static double check(double value, double min, double max, String name) {
        if (!(Double.isFinite(value) && value >= min && value <= max)) {
            throw new IllegalArgumentException(name + " must be between " + min + " and " + max + ", got " + value);
        }
        return value;
    }
}
