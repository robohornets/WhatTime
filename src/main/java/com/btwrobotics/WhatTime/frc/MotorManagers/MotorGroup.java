package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Groups multiple Motor instances and applies the same actions to all of them.
 */
public class MotorGroup extends SubsystemBase {
    private final List<Motor> motors;

    public MotorGroup(List<Motor> motors) {
        if (motors == null || motors.isEmpty()) {
            throw new IllegalArgumentException("motors cannot be empty.");
        }

        this.motors = motors;
    }

    public boolean toggleEnabled() {
        return toggleEnabled(null);
    }

    public boolean toggleEnabled(Boolean enabled) {
        boolean allEnabled = true;
        for (Motor motor : motors) {
            allEnabled &= motor.toggleEnabled(enabled);
        }
        return allEnabled;
    }

    public boolean isEnabled() {
        for (Motor motor : motors) {
            if (!motor.isEnabled()) {
                return false;
            }
        }
        return true;
    }

    public void drive() {
        for (Motor motor : motors) {
            motor.drive();
        }
    }

    public void drive(double speed) {
        for (Motor motor : motors) {
            motor.drive(speed);
        }
    }

    public void goTo(double target) {
        for (Motor motor : motors) {
            motor.goTo(target);
        }
    }

    public void setTarget(double target) {
        goTo(target);
    }

    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        for (Motor motor : motors) {
            motor.setNeutralMode(neutralModeValue);
        }
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

    public List<Motor> getMotors() {
        return motors;
    }
}
