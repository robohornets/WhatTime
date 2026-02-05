package com.btwrobotics.WhatTime.frc.DashboardManagers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Utility class for simplified interaction with NetworkTables.
 * 
 * <p>This class provides a convenient API for publishing data to NetworkTables,
 * which is the primary mechanism for communication between the robot code and
 * the driver station dashboard. It supports multiple data types and allows
 * publishing to both a default table and custom-named tables.
 * 
 * <p>The default table name is "CustomDashboard", but methods are provided
 * to publish to any arbitrary table.
 * 
 * @see edu.wpi.first.networktables.NetworkTable
 * @see edu.wpi.first.networktables.NetworkTableInstance
 */
public class NetworkTablesUtil {
    /** The default NetworkTables instance used for all operations. */
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    
    /** The default table name used when no specific table is specified. */
    private static final String DEFAULT_TABLE = "CustomDashboard";



    /**
     * Gets the default NetworkTable.
     * 
     * <p>Returns a reference to the table named "CustomDashboard".
     * 
     * @return the default NetworkTable instance
     */
    public static NetworkTable getTable() {
        return inst.getTable(DEFAULT_TABLE);
    }

    /**
     * Gets a NetworkTable by name.
     * 
     * <p>Returns a reference to a NetworkTable with the specified name.
     * If the table does not exist, it will be created.
     * 
     * @param table the name of the table to retrieve
     * @return the NetworkTable instance with the specified name
     */
    public static NetworkTable getTable(String table) {
        return inst.getTable(table);
    }
    
    /**
     * Publishes a numeric value to the default table.
     * 
     * @param key the entry name/key within the table
     * @param value the double value to publish
     */
    public static void put(String key, double value) {
        inst.getTable(DEFAULT_TABLE).getEntry(key).setDouble(value);
    }

    /**
     * Publishes a string value to the default table.
     * 
     * @param key the entry name/key within the table
     * @param value the string value to publish
     */
    public static void put(String key, String value) {
        inst.getTable(DEFAULT_TABLE).getEntry(key).setString(value);
    }

    /**
     * Publishes a boolean value to the default table.
     * 
     * @param key the entry name/key within the table
     * @param value the boolean value to publish
     */
    public static void put(String key, boolean value) {
        inst.getTable(DEFAULT_TABLE).getEntry(key).setBoolean(value);
    }

    /**
     * Publishes a generic object to the default table.
     * 
     * <p>This method handles type conversion automatically:
     * <ul>
     *   <li>Numbers are converted to doubles
     *   <li>Booleans are stored as booleans
     *   <li>All other objects are converted to strings via toString()
     * </ul>
     * 
     * @param key the entry name/key within the table
     * @param value the object to publish
     */
    public static void put(String key, Object value) {
        if (value instanceof Number) {
            inst.getTable(DEFAULT_TABLE).getEntry(key).setDouble(((Number) value).doubleValue());
        } else if (value instanceof Boolean) {
            inst.getTable(DEFAULT_TABLE).getEntry(key).setBoolean((Boolean) value);
        } else {
            inst.getTable(DEFAULT_TABLE).getEntry(key).setString(value.toString());
        }
    }
    
    /**
     * Publishes a numeric value to a specified table.
     * 
     * @param table the name of the table to publish to
     * @param key the entry name/key within the table
     * @param value the double value to publish
     */
    public static void put(String table, String key, double value) {
        inst.getTable(table).getEntry(key).setDouble(value);
    }

    /**
     * Publishes a string value to a specified table.
     * 
     * @param table the name of the table to publish to
     * @param key the entry name/key within the table
     * @param value the string value to publish
     */
    public static void put(String table, String key, String value) {
        inst.getTable(table).getEntry(key).setString(value);
    }

    /**
     * Publishes a boolean value to a specified table.
     * 
     * @param table the name of the table to publish to
     * @param key the entry name/key within the table
     * @param value the boolean value to publish
     */
    public static void put(String table, String key, boolean value) {
        inst.getTable(table).getEntry(key).setBoolean(value);
    }

    /**
     * Publishes a generic object to a specified table.
     * 
     * <p>This method handles type conversion automatically:
     * <ul>
     *   <li>Numbers are converted to doubles
     *   <li>Booleans are stored as booleans
     *   <li>All other objects are converted to strings via toString()
     * </ul>
     * 
     * @param table the name of the table to publish to
     * @param key the entry name/key within the table
     * @param value the object to publish
     */
    public static void put(String table, String key, Object value) {
        if (value instanceof Number) {
            inst.getTable(DEFAULT_TABLE).getEntry(key).setDouble(((Number) value).doubleValue());
        } else if (value instanceof Boolean) {
            inst.getTable(DEFAULT_TABLE).getEntry(key).setBoolean((Boolean) value);
        } else {
            inst.getTable(DEFAULT_TABLE).getEntry(key).setString(value.toString());
        }
    }
}