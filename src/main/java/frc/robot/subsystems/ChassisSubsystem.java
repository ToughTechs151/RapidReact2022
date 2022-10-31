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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverOI;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class ChassisSubsystem extends SubsystemBase {

  public enum RobotSide {
    LEFT,
    RIGHT
  }

  static final String LEFTSPEED = "Left Speed";
  static final String RIGHTSPEED = "Right Speed";

  // Motor declarations
  private CANSparkMax frontLeftMotor =
      new CANSparkMax(Constants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax frontRightMotor =
      new CANSparkMax(Constants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rearLeftMotor =
      new CANSparkMax(Constants.REAR_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rearRightMotor =
      new CANSparkMax(Constants.REAR_RIGHT_MOTOR, MotorType.kBrushless);
  // The gyro sensor
  private final ADXRS450_Gyro mmgyro = new ADXRS450_Gyro();
  private final PIDController controller =
      new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);
  // Ultrasonic sensor
  private final AnalogInput rightUltrasonic = new AnalogInput(0);
  private final AnalogInput leftUltrasonic = new AnalogInput(1);
  public double ultrasonicSensorRange = 0;
  public double voltageScaleFactor = 1;

  // Encoder declarations
  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder rearLeftEncoder;
  private RelativeEncoder rearRightEncoder;

  private DifferentialDrive driveTrain;
  private double mmgyroAngle = Constants.GYRO_NOTUSED;
  private PhotonCamera mmcamera;
  private boolean mmAimBall = false;
  private double yaw;

  // drive constants
  /** The scaling factor between the joystick value and the speed controller. */
  private double speedMultiplier = 0.5;

  /** The scale factor for normal mode. */
  private static final double NORMAL = 1.0;

  /** The scale factor for crawl mode. */
  private static final double CRAWL = 0.3;

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
    mmgyroAngle = Constants.GYRO_NOTUSED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Use these to plot the input vs RPM onto a plot
    SmartDashboard.putNumber("Left Encoder", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", frontRightEncoder.getPosition());
    SmartDashboard.putNumber("Gyro angle", mmgyro.getAngle());
    SmartDashboard.putNumber("Left Ultrasonic", getLeftUltrasonic());
    SmartDashboard.putNumber("Right Ultrasonic  ", getRightUltrasonic());
  }

  /**
   * tankDrive - drive the chassis using tank drive method.
   *
   * @param leftSpeed The left joystick controller spped -1 to 1
   * @param rightSpeed The right joystick controller speed -1 to 1
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed, true);
  }

  /**
   * arcadeDrive - drive the chassis using arcade drive method.
   *
   * @param speed The speed of the motor.
   * @param rotation The rate of rotation.
   */
  public void arcadeDrive(double speed, double rotation) {
    driveTrain.arcadeDrive(speed, rotation);
  }

  /** Start driving straight. */
  public void startDriveStraight() {
    resetGyro();
    mmgyroAngle = mmgyro.getAngle();
    controller.setTolerance(1, 5);
  }

  /** Stop driving straight. */
  public void endDriveStraight() {
    mmgyroAngle = Constants.GYRO_NOTUSED;
  }

  /** Starting aiming the red ball. */
  public void startAimRedBall() {
    mmcamera.setLED(VisionLEDMode.kOn);
    mmcamera.setPipelineIndex(0); // This is the first pipeline from http://gloworm.local:5800
    yaw = 0;
    mmAimBall = true;
    controller.setTolerance(1, 5);
  }

  /** Start aiming the blue ball. */
  public void startAimBlueBall() {
    mmcamera.setLED(VisionLEDMode.kOn);
    mmcamera.setPipelineIndex(1); // This is the second pipeline from http://gloworm.local:5800
    yaw = 0;
    mmAimBall = true;
    controller.setTolerance(1, 5);
  }

  /** Stop aiming the ball. */
  public void endAimBall() {
    mmcamera.setLED(VisionLEDMode.kOff);
    mmAimBall = false;
  }

  /** Turn on the LED. */
  public void startLedOn() {
    mmcamera.setLED(VisionLEDMode.kOn);
  }

  /** Turn off the LED. */
  public void startLedOff() {
    mmcamera.setLED(VisionLEDMode.kOff);
  }

  /**
   * drive - set speed to chassis with the joystick input with a scale.
   *
   * @param driverOI The controller to use to drive.
   * @param scale What scale factor to apply.
   */
  public void drive(RobotContainer robotContainer, DriverOI driverOI, int scale) {

    mmcamera = robotContainer.getCamera();
    speedMultiplier = driverOI.getJoystick().getRawButton(Constants.RIGHT_BUMPER) ? CRAWL : NORMAL;

    RobotContainer.DriveTrainType driveTrainType = robotContainer.getDriveTrainType();

    if (driveTrainType == RobotContainer.DriveTrainType.TANK) {
      double rightVal;
      double leftVal;
      if (dir == Constants.INTAKE_IN) {
        rightVal = driverOI.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y);
        leftVal = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y);
      } else {
        rightVal = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y);
        leftVal = driverOI.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y);
      }

      if (mmgyroAngle != Constants.GYRO_NOTUSED) {
        var pidOutput = controller.calculate(getGyroAngle(), 0) / 10;

        var speed = leftVal * speedMultiplier * dir;
        SmartDashboard.putNumber(LEFTSPEED, speed + pidOutput);
        SmartDashboard.putNumber(RIGHTSPEED, speed - pidOutput);
        tankDrive(speed + pidOutput, speed - pidOutput);
      } else if (mmAimBall) {
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        var result = mmcamera.getLatestResult();
        if (result.hasTargets()) {
          yaw = result.getBestTarget().getYaw();
          SmartDashboard.putNumber("Yaw", yaw);
        }

        var pidOutput = controller.calculate(-yaw, 0) / 2;
        // Set minimum voltage for motors
        if (pidOutput < 0) {
          pidOutput = Math.min(pidOutput, -0.3);
        } else {
          pidOutput = Math.max(pidOutput, 0.3);
        }
        SmartDashboard.putNumber("TurnPID", pidOutput);

        var speed = leftVal * dir;
        SmartDashboard.putNumber(LEFTSPEED, speed + pidOutput);
        SmartDashboard.putNumber(RIGHTSPEED, speed - pidOutput);
        tankDrive(speed + pidOutput, speed - pidOutput);
      } else {
        SmartDashboard.putNumber(LEFTSPEED, leftVal * speedMultiplier * dir);
        SmartDashboard.putNumber(RIGHTSPEED, rightVal * speedMultiplier * dir);
        tankDrive(leftVal * speedMultiplier * dir, rightVal * speedMultiplier * dir);
      }
    } else if (driveTrainType == RobotContainer.DriveTrainType.ARCADE) {
      double speed = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y) * dir;
      double rotation = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_X);
      arcadeDrive(speed, rotation);
    }
  }

  public double getGyroAngle() {
    return mmgyro.getAngle();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    mmgyro.reset();
  }

  public void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
  }

  /** Get the position of the left encoder in inches. */
  public double getLeftDistanceInch() {
    return frontLeftEncoder.getPosition()
        / Constants.DRIVE_GEAR_RATIO
        * Math.PI
        * Constants.DRIVE_WHEEL_DIAMETER;
  }

  /** Get the position of the right encoder in inches. */
  public double getRightDistanceInch() {
    return frontRightEncoder.getPosition()
        / Constants.DRIVE_GEAR_RATIO
        * Math.PI
        * Constants.DRIVE_WHEEL_DIAMETER;
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  private double getRightUltrasonic() {
    voltageScaleFactor =
        5
            / RobotController
                .getVoltage5V(); // Calculate what percentage of 5 Volts we are actually at
    // Get a reading from the first sensor, scale it by the voltageScaleFactor, and then scale to
    // Inches
    ultrasonicSensorRange = rightUltrasonic.getValue() * voltageScaleFactor * 0.0492;
    if (ultrasonicSensorRange < 18) {
      ultrasonicSensorRange = 195;
    }
    return ultrasonicSensorRange;
  }

  private double getLeftUltrasonic() {
    voltageScaleFactor =
        5
            / RobotController
                .getVoltage5V(); // Calculate what percentage of 5 Volts we are actually at
    // Get a reading from the first sensor, scale it by the voltageScaleFactor, and then scale to
    // Inches
    ultrasonicSensorRange = leftUltrasonic.getValue() * voltageScaleFactor * 0.0492;
    if (ultrasonicSensorRange < 18) {
      ultrasonicSensorRange = 195;
    }
    return ultrasonicSensorRange;
  }

  public double getUltrasonic() {
    return Math.min(getLeftUltrasonic(), getRightUltrasonic());
  }
}
