// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import io.github.oblarg.oblog.annotations.Log;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Robot preferences
    public static final String DRIVE_TRAIN_TYPE = "DriveTrainType";
    public static final String TANK = "Tank";
    public static final String ARCADE = "Arcade";    

    // Scale Factor
    public static final int SCALED_DRIVE = 2;

    // Directions
    public static final int FORWARD = 1;
    public static final int REVERSE = -1;

    // Position
    public static final int UP = 1;
    public static final int DOWN = 0;

    // Drive Train CAN IDs
    public static int FRONT_LEFT_MOTOR = 4;
    public static int FRONT_RIGHT_MOTOR = 3;
    public static int REAR_LEFT_MOTOR = 1;
    public static int REAR_RIGHT_MOTOR = 2;

    // Mechanism CAN IDs
    public static int INTAKE_MOTOR = 6;
    public static int ARM_MOTOR = 5;
    //Intake Subsystem Speed
    public static final double INTAKE_SPEED = 0.5;

    // OI ports
    public static int DRIVER_OI = 0;
    public static int CODRIVE_OI = 1;

    //OI AXES
    public static final int LEFT_JOYSTICK_X = 0;
    public static final int LEFT_JOYSTICK_Y = 1;
    public static final int RIGHT_JOYSTICK_X = 4;
    public static final int RIGHT_JOYSTICK_Y = 5;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int RIGHT_JOYSTICK_VERTICAL_AXIS = 5;
    
    //OI BUTTON PORTS
	public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
	public static final int Y = 4;
	public static final int LEFT_BUMPER = 5;
	public static final int RIGHT_BUMPER = 6;
	public static final int BACK = 7;
	public static final int START = 8;
	public static final int LEFT_JOYSTICK = 9;
    public static final int RIGHT_JOYSTICK = 10;

    //public static final String Constants = null;    

     // Original Gains
    @Log
    public static double LAUNCHERKP=0.0003;
    @Log
    public static double LAUNCHERKI=0.00032;
    @Log
    public static double LAUNCHERKD=0.000000003;
}
