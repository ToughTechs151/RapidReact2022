
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
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
  private final PIDController controller = new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);
  // Ultrasonic sensor
  private final AnalogInput ultrasonic = new AnalogInput(0);
  public DigitalOutput ultrasonicTriggerPin = new DigitalOutput(0);
  public double ultrasonicSensorRange = 0;
  public double voltageScaleFactor = 1; 

  // Encoder declarations
  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder rearLeftEncoder;
  private RelativeEncoder rearRightEncoder;

  private DifferentialDrive driveTrain;
  private double m_gyroAngle = Constants.GYRO_NOTUSED;

  //drive constants
  /**
  * The scaling factor between the joystick value and the speed controller
  */
  private double speedMultiplier = 0.5;

  /**
   * The scale factor for normal mode
   */
  private static final double normal = 1.0;

  /**
   * The scale factor for crawl mode
   */
  private static final double crawl = 0.3;

  /**
   * The direction which is "forward"; 1 represents the hatch side and -1 represents the cargo side.
   */
  private int dir = Constants.INTAKE_IN;

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

    double rampRate = Constants.RAMP_RATE;
    frontLeftMotor.setOpenLoopRampRate(rampRate);
    frontRightMotor.setOpenLoopRampRate(rampRate);
    
    rearLeftMotor.follow(frontLeftMotor);
    rearRightMotor.follow(frontRightMotor);

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    rearLeftEncoder = rearLeftMotor.getEncoder();
    rearRightEncoder = rearRightMotor.getEncoder();

    driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    frontRightMotor.setInverted(true);
    m_gyroAngle = Constants.GYRO_NOTUSED;
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
    SmartDashboard.putNumber("Sensor Range", getUltrasonicDistanceInch());
  }

  /**
   * tankDrive - drive the chassis using tank drive method
   * @param leftspeed - The left joystick controller spped -1 to 1
   * @param rightspeed - The right joystick controller speed -1 to 1
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed, true);
  }

  /**
   * arcadeDrive - drive the chassis using arcade drive method
   * @param speed
   * @param rotation
   */
  public void arcadeDrive(double speed, double rotation) {
    driveTrain.arcadeDrive(speed, rotation);
  }

  public void startDriveStraight() {
    resetGyro();
    m_gyroAngle = m_gyro.getAngle();
    controller.setTolerance(1, 5);
  }
  public void endDriveStraight() {
    m_gyroAngle = Constants.GYRO_NOTUSED;
  }

  /**
   * drive - set speed to chassis with the joystick input with a scale
   * @param driverOI
   * @param scale
   */
  public void drive(RobotContainer robotContainer, DriverOI driverOI, int scale) {
    speedMultiplier = driverOI.getJoystick().getRawButton(Constants.RIGHT_BUMPER) ? crawl : normal;
    // LEFT_BUMPER is now changed to drive straight tankDrive.
    // dir = driverOI.getJoystick().getRawButton(Constants.LEFT_BUMPER) ? Constants.INTAKE_OUT : Constants.INTAKE_IN;
    
    RobotContainer.DriveTrainType driveTrainType = robotContainer.getDriveTrainType();

    if (driveTrainType == RobotContainer.DriveTrainType.TANK) {
      double rightVal;
      double leftVal;
      if(dir == Constants.INTAKE_IN) {
        rightVal = driverOI.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y);
        leftVal = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y);
      } else {
        rightVal = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y);
        leftVal = driverOI.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y);
      }
      if(m_gyroAngle != Constants.GYRO_NOTUSED)
      {
        var pidOutput = controller.calculate(getGyroAngle(), 0) / 10;

        var speed = leftVal * speedMultiplier * dir;
        SmartDashboard.putNumber("Left Speed", speed+pidOutput);
        SmartDashboard.putNumber("Right Speed", speed-pidOutput);
        tankDrive(speed + pidOutput, speed - pidOutput);
      }
      else
      {
        SmartDashboard.putNumber("Left Speed", leftVal);
        SmartDashboard.putNumber("Right Speed", rightVal);
        tankDrive(leftVal * speedMultiplier * dir, rightVal * speedMultiplier * dir);
      }
    } else if (driveTrainType == RobotContainer.DriveTrainType.ARCADE) {
      double speed = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y) * dir;
      double rotation = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_X);
      SmartDashboard.putNumber("Speed", speed);
      SmartDashboard.putNumber("Rotation", rotation);
      arcadeDrive(speed, rotation);
    }
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

  public double getUltrasonicDistanceInch() {
    voltageScaleFactor = 5/RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at
    //Get a reading from the first sensor, scale it by the voltageScaleFactor, and then scale to Inches
    ultrasonicSensorRange = ultrasonic.getValue()*voltageScaleFactor*0.0492;
    return ultrasonicSensorRange;
  }
}
