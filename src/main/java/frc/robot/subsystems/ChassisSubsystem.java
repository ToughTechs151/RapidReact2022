// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverOI;

public class ChassisSubsystem extends SubsystemBase {
  public enum RobotSide {
    LEFT, RIGHT;
  }

  // Motor declarations
  private CANSparkMax frontLeftMotor = new CANSparkMax(Constants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(Constants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rearLeftMotor = new CANSparkMax(Constants.REAR_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rearRightMotor = new CANSparkMax(Constants.REAR_RIGHT_MOTOR, MotorType.kBrushless);  
  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Encoder declarations
  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder rearLeftEncoder;
  private RelativeEncoder rearRightEncoder;

  private DifferentialDrive driveTrain;
  
  //drive constants
  /**
  * The scaling factor between the joystick value and the speed controller
  */
  private double speedMultiplier = 1.0;

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
  private int dir = Constants.IN;

  /** Creates a new ChassisSubsystem. */
  public ChassisSubsystem() {

    frontLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    rearLeftMotor.restoreFactoryDefaults();
    rearRightMotor.restoreFactoryDefaults();

    frontLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMITS);
    frontRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMITS);
    rearLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMITS);
    rearRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMITS);

    frontLeftMotor.setIdleMode(IdleMode.kCoast);
    frontRightMotor.setIdleMode(IdleMode.kCoast);
    rearLeftMotor.setIdleMode(IdleMode.kCoast);
    rearRightMotor.setIdleMode(IdleMode.kCoast);

    rearLeftMotor.follow(frontLeftMotor);
    rearRightMotor.follow(frontRightMotor);

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    rearLeftEncoder = rearLeftMotor.getEncoder();
    rearRightEncoder = rearRightMotor.getEncoder();

    driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    frontRightMotor.setInverted(true);    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Use these to plot the input vs RPM onto a plot
    SmartDashboard.putNumber("Front Left Motor RPM", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Front Right Motor RPM", frontRightEncoder.getVelocity());
    SmartDashboard.putNumber("Rear Left Motor RPM", rearLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Rear Right Motor RPM", rearRightEncoder.getVelocity());
    SmartDashboard.putNumber("Left  POS", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Right POS", frontRightEncoder.getPosition());
    SmartDashboard.putNumber("Gyro angle", m_gyro.getAngle());    
  }

  /**
   * tankDrive - drive the chassis using tank drive method
   * @param leftspeed - The left joystick controller spped -1 to 1
   * @param rightspeed - The right joystick controller speed -1 to 1
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * arcadeDrive - drive the chassis using arcade drive method
   * @param speed
   * @param rotation
   */
  public void arcadeDrive(double speed, double rotation) {
    driveTrain.arcadeDrive(speed, rotation);
  }

  /**
   * drive - set speed to chassis with the joystick input with a scale
   * @param driverOI
   * @param scale
   */
  public void drive(RobotContainer robotContainer, DriverOI driverOI, int scale) {
    speedMultiplier = driverOI.getJoystick().getRawButton(Constants.RIGHT_BUMPER) ? crawl : normal;
    dir = driverOI.getJoystick().getRawButton(Constants.LEFT_BUMPER) ? Constants.OUT : Constants.IN;
    
    RobotContainer.DriveTrainType driveTrainType = robotContainer.getDriveTrainType();

    if (driveTrainType == RobotContainer.DriveTrainType.TANK) {
      double rightVal;  
      double leftVal;
      if(dir == Constants.IN) {
        rightVal = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y), scale, RobotSide.RIGHT);
        leftVal = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y), scale, RobotSide.LEFT);
      } else {
        rightVal = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y), scale, RobotSide.RIGHT);
        leftVal = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y), scale, RobotSide.LEFT);
      }
      SmartDashboard.putNumber("Left Speed", leftVal);
      SmartDashboard.putNumber("Right Speed", rightVal);
      tankDrive(leftVal * speedMultiplier * dir, rightVal * speedMultiplier * dir);
    } else if (driveTrainType == RobotContainer.DriveTrainType.ARCADE) {
      double speed = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y), scale, RobotSide.LEFT) * dir;
      double rotation = getScaledValue(driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_X), scale, RobotSide.RIGHT);
      SmartDashboard.putNumber("Speed", speed);
      SmartDashboard.putNumber("Rotation", rotation);
      arcadeDrive(speed, rotation);
    }
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

  public double getGyroAngle() {
    return m_gyro.getAngle();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  public void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
  }

  public double getLeftDistanceInch() {
    return frontLeftEncoder.getPosition() / Constants.DRIVE_GEAR_RATIO * Math.PI * Constants.DRIVE_WHEEL_DIAMETER;
    
  }

  public double getRightDistanceInch() {
    return frontRightEncoder.getPosition() / Constants.DRIVE_GEAR_RATIO * Math.PI * Constants.DRIVE_WHEEL_DIAMETER;
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }
}
