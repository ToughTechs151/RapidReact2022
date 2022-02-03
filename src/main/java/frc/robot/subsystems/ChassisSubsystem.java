// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.oi.DriverOI;

public class ChassisSubsystem extends SubsystemBase {
  public enum RobotSide {
    LEFT, RIGHT;
  }

  // Motor declarations
  private CANSparkMax frontLeftMotor_ = new CANSparkMax(Constants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax frontRightMotor_ = new CANSparkMax(Constants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rearLeftMotor_ = new CANSparkMax(Constants.REAR_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rearRightMotor_ = new CANSparkMax(Constants.REAR_RIGHT_MOTOR, MotorType.kBrushless);

  // Encoder declarations
  private RelativeEncoder frontLeftEncoder_ = null;
  private RelativeEncoder frontRightEncoder_ = null;
  private RelativeEncoder rearLeftEncoder_ = null;
  private RelativeEncoder rearRightEncoder_ = null;

  private MotorControllerGroup leftMotors_ = new MotorControllerGroup(frontLeftMotor_, rearLeftMotor_);
  private MotorControllerGroup rightMotors_ = new MotorControllerGroup(frontRightMotor_, rearRightMotor_);

  private DifferentialDrive driveTrain_ = new DifferentialDrive(leftMotors_, rightMotors_);

  //drive constants
  /**
  * The scaling factor between the joystick value and the speed controller
  */
  private static double speedMultiplier = 1.0;

  /**
   * The scale factor for normal mode
   */
  private static final double normal = 1.0;

  /**
   * The scale factor for crawl mode
   */
  private static final double crawl = 0.5;  

    /**
   * The minimum (closest to 0) speed controller command for the right side of the drive train to start moving forward. Must be empirically derived.
   */
  private static double mechDeadbandRightForward = 0.25;

  /**
   * The maximum (closest to 0) speed controller command for the right side of the drive train to start moving backward. Must be empirically derived.
   * Must be negative.
   */
  private static double mechDeadbandRightBackward = -0.27;

  /**
   * The minimum (closest to 0) speed controller command for the left side of the drive train to start moving forward. Must be empirically derived.
   */
  private static double mechDeadbandLeftForward = 0.23;

  /**
   * The maximum (closest to 0) speed controller command for the left side of the drive train to start moving backward. Must be empirically derived.
   * Must be negative.
   */
  private static double mechDeadbandLeftBackward = -0.29;

  /**
   * The minimum joystick value to actually send a command to the speed controller, to prevent noise near 0.
   */
  private static double softwareDeadband = 0.05;

  //arcade drive constant
  private double turnGain = 0.75;
  
  /**
   * The direction which is "forward"; 1 represents the hatch side and -1 represents the cargo side.
   */
  private int dir = Constants.REVERSE;

  /** Creates a new ChassisSubsystem. */
  public ChassisSubsystem() {
    rightMotors_.setInverted(true);
    frontLeftMotor_.setIdleMode(IdleMode.kCoast);
    frontRightMotor_.setIdleMode(IdleMode.kCoast);
    rearLeftMotor_.setIdleMode(IdleMode.kCoast);
    rearRightMotor_.setIdleMode(IdleMode.kCoast);

    frontLeftEncoder_ = frontLeftMotor_.getEncoder();
    frontRightEncoder_ = frontRightMotor_.getEncoder();
    rearLeftEncoder_ = rearLeftMotor_.getEncoder();
    rearRightEncoder_ = rearRightMotor_.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /**
   * 
   * @param leftspeed - The left joystick controller spped -1 to 1
   * @param rightspeed - The right joystick controller speed -1 to 1
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain_.tankDrive(leftSpeed, rightSpeed);
  }

  public void drive(DriverOI driverOI, int scale) {
    speedMultiplier = driverOI.getJoystick().getRawButton(Constants.RIGHT_BUMPER) ? crawl : normal;
    dir = driverOI.getJoystick().getRawButton(Constants.LEFT_BUMPER) ? Constants.FORWARD : Constants.REVERSE;

    double rightVal = 0;
    double leftVal = 0;

    if(dir == Constants.REVERSE) {
        rightVal = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y), scale, RobotSide.RIGHT);
        leftVal = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y), scale, RobotSide.LEFT);
    } else if(dir == Constants.FORWARD) {
        rightVal = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y), scale, RobotSide.RIGHT);
        leftVal = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y), scale, RobotSide.LEFT);
    }
    
    tankDrive(leftVal * speedMultiplier * dir, rightVal * speedMultiplier * dir);
  }

  /**
   * Find the scale factor based on the input value and scale on each side of the Robot's motors
   * 
   * @param val
   * @param scale
   * @param side
   * @return
   */
  private double getScaledValue(double val, int scale, RobotSide side) {
    double forward, back;

    if(side.equals(RobotSide.RIGHT)) {
        forward = Math.abs(mechDeadbandRightForward);
        back = Math.abs(mechDeadbandRightBackward);
    } else {
        forward = Math.abs(mechDeadbandLeftForward);
        back = Math.abs(mechDeadbandLeftBackward);
    }

    if(Math.abs(val) < softwareDeadband) {
        return 0;
    } else if(scale == 0) {
        return deadzone(val);
    } else if(scale == 1) {
        if(val > 0) {
            return (((1.0 - forward) * (val - 1.0) / (1.0 - softwareDeadband)) + 1.0);
        } else {
            return (((1.0 - back) * (val + 1.0) / (1.0 - softwareDeadband)) -  1.0);
        }
    } else if(scale == 2) {
        if(val > 0) {
            return (1- forward) / Math.pow(1 - softwareDeadband, 2) * Math.pow(val - softwareDeadband, 2) + forward;
        } else {
            return (-1 + back) / Math.pow(-1 + softwareDeadband, 2) * Math.pow(val + softwareDeadband, 2) - back;
        }
    } else {
        return 0;
    }
  }

  /**
   * Method to reduce noise around the 0 position of the joystick.
   * @param val The raw joystick input.
   * @return If the input is outside the range <code> (-softwareDeadband < val < softwareDeadband)</code>, it is returned. Else, 0 is returned.
   */
  private double deadzone(double val) {
    return Math.abs(val) > softwareDeadband ? val : 0;
  }
}
