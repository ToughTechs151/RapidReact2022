// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 4;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 2;

        public static final int[] kLeftEncoderPorts = new int[] {4, 1};
        public static final int[] kRightEncoderPorts = new int[] {3, 2};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = true;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 1.5;
        public static final double kaVoltSecondsSquaredPerRadian = 0.3;

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
            LinearSystemId.identifyDrivetrainSystem(
                kvVoltSecondsPerMeter,
                kaVoltSecondsSquaredPerMeter,
                kvVoltSecondsPerRadian,
                kaVoltSecondsSquaredPerRadian);

        // Example values only -- use what's on your physical robot!
        public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
        public static final double kDriveGearing = 8;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
    // Robot preferences
    public static final String DRIVE_TRAIN_TYPE = "DriveTrainType";
    public static final String TANK = "Tank";
    public static final String ARCADE = "Arcade";

    // Scale Factor
    public static final int SCALED_DRIVE = 0;
    public static final double RAMP_RATE = 0.0;

    // Directions
    public static final int INTAKE_OUT = 1;
    public static final int INTAKE_IN = -1;

    // Position
    public static final double ARM_UP = -0.5;
    public static final double ARM_DOWN = -18;

    // Camera ID
    public static final int CAMERA_0 = 0;
    public static final int CAMERA_1 = 1;

    // Drive Train CAN IDs
    public static int FRONT_LEFT_MOTOR = 4;
    public static int FRONT_RIGHT_MOTOR = 3;
    public static int REAR_LEFT_MOTOR = 1;
    public static int REAR_RIGHT_MOTOR = 2;

    // Mechanism CAN IDs
    public static int INTAKE_MOTOR = 6;
    public static int ARM_MOTOR = 5;

    public static final int MOTOR_CURRENT_LIMITS = 30;

    // OI ports
    public static int DRIVER_OI = 0;
    public static int CODRIVER_OI = 1;

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

    // Arm Values
    public static final double ARM_REVOLUTION = 4.5;
    public static final double POS_TOLERANCE = 0.005;
    public static final double ARM_KP = 0.05;
    public static final double ARM_KI = 0.0;
    public static final double ARM_KD = 0.0;
    public static final double DRIVE_GEAR_RATIO = 10.71;
    public static final double DRIVE_WHEEL_DIAMETER = 6;

    public static final int GYRO_NOTUSED = -1;

    //PID Value
    public static final double DRIVETRAIN_KP = 0.03;
    public static final double DRIVETRAIN_KI = 0;
    public static final double DRIVETRAIN_KD = 0;
    public static final double LIMELIGHT_KP = 0.02;
}
