package com.btwrobotics.WhatTime.frc.MotorManagers;

import java.util.List;

/**
 * Utility class for handling motor position calculations.
 * 
 * <p>This class provides methods for analyzing and computing position-related
 * values across multiple motors, such as calculating average positions for
 * synchronized motor groups.
 * 
 * @see MotorWrapper
 */
public class MotorPositionHandler {
    /**
     * Calculates the average position across multiple motors.
     * 
     * <p>This method computes the arithmetic mean of positions from all motors
     * in the provided list. This is useful for mechanisms where multiple motors
     * should maintain synchronized positions, such as dual-motor elevators or
     * drive train sides.
     * 
     * <p>If the list is empty, returns π (pi) as a sentinel value to indicate
     * an error condition.
     * 
     * @param motors the list of motor wrappers to average positions from
     * @return the average position in rotations, or π if the list is empty
     */
    public double averagePositions(List<MotorWrapper> motors) {
        double addingValue = 0.0;
        for (MotorWrapper motor : motors) {
            addingValue += motor.getPosition();
        }

        // If motors is not empty
        if (motors.size() != 0.0) {
            return addingValue / motors.size();
        }
        // else return a default value
        else {
            return 3.141592;
        }
    }
}
