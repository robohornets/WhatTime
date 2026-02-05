package com.btwrobotics.WhatTime.frc.DashboardManagers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class for simplified interaction with Shuffleboard/SmartDashboard.
 * 
 * <p>This class provides a convenient API for publishing data to SmartDashboard,
 * which is displayed in the Shuffleboard application on the driver station.
 * It supports multiple data types and provides overloaded methods for ease of use.
 * 
 * <p>All methods delegate to {@link SmartDashboard} for the actual data publishing.
 * 
 * @see edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
 */
public class ShuffleboardUtil {
    /**
     * Publishes a numeric value to SmartDashboard.
     * 
     * @param key the entry name/key to display in Shuffleboard
     * @param value the double value to publish
     */
    public static void put(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    /**
     * Publishes a string value to SmartDashboard.
     * 
     * @param key the entry name/key to display in Shuffleboard
     * @param value the string value to publish
     */
    public static void put(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    /**
     * Publishes a boolean value to SmartDashboard.
     * 
     * @param key the entry name/key to display in Shuffleboard
     * @param value the boolean value to publish
     */
    public static void put(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    /**
     * Publishes a generic object to SmartDashboard.
     * 
     * <p>This method handles type conversion automatically:
     * <ul>
     *   <li>Numbers are converted to doubles and published via putNumber
     *   <li>Booleans are published via putBoolean
     *   <li>All other objects are converted to strings via toString() and published via putString
     * </ul>
     * 
     * @param key the entry name/key to display in Shuffleboard
     * @param value the object to publish
     */
    public static void put(String key, Object value) {
        if (value instanceof Number) {
            SmartDashboard.putNumber(key, ((Number) value).doubleValue());
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (Boolean) value);
        } else {
            SmartDashboard.putString(key, value.toString());
        }
    }
}