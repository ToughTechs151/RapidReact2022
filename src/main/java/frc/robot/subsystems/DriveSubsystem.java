// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.PhotonCamera;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless),
          new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
          new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless),
          new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless));

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private final PIDController controller = new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);
  // Ultrasonic sensor
  private final AnalogInput rightUltrasonic = new AnalogInput(0);
  private final AnalogInput leftUltrasonic = new AnalogInput(1);
  public double ultrasonicSensorRange = 0;
  public double voltageScaleFactor = 1;

  private double m_gyroAngle = Constants.GYRO_NOTUSED;
  private PhotonCamera m_camera;
  private boolean m_AimBall = false;
  private double yaw;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;
  private ADXRS450_GyroSim m_gyroSim;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      m_gyroSim = new ADXRS450_GyroSim(m_gyro);
      m_gyroAngle = Constants.GYRO_NOTUSED;

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
    m_fieldSim.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(
        m_leftMotors.get() * RobotController.getBatteryVoltage(),
        m_rightMotors.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   *
   * @param leftspeed - The left joystick controller spped -1 to 1
   * @param rightspeed - The right joystick controller speed -1 to 1
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    if(m_gyroAngle != Constants.GYRO_NOTUSED)
    {
      var pidOutput = controller.calculate(getGyroAngle(), 0) / 10;
      SmartDashboard.putNumber("Left Speed", leftSpeed+pidOutput);
      SmartDashboard.putNumber("Right Speed", rightSpeed-pidOutput);
      m_drive.tankDrive(leftSpeed + pidOutput, rightSpeed - pidOutput);
    }
    else if(m_AimBall)
    {
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = m_camera.getLatestResult();
      if (result.hasTargets()) {
        yaw = result.getBestTarget().getYaw();
        SmartDashboard.putNumber("Yaw", yaw);
      }

      var pidOutput = controller.calculate(-yaw, 0) / 2;
      // Set minimum voltage for motors
      if(pidOutput < 0) {
        pidOutput = Math.min(pidOutput, -0.3);
      } else {
        pidOutput = Math.max(pidOutput, 0.3);
      }
      SmartDashboard.putNumber("TurnPID", pidOutput);

      var speed = leftSpeed;
      SmartDashboard.putNumber("Left Speed", speed+pidOutput);
      SmartDashboard.putNumber("Right Speed", speed-pidOutput);
      m_drive.tankDrive(leftSpeed + pidOutput, rightSpeed - pidOutput);
    }
    else
    {
      SmartDashboard.putNumber("Left Speed", leftSpeed);
      SmartDashboard.putNumber("Right Speed", rightSpeed);
      m_drive.tankDrive(leftSpeed, rightSpeed);
    }
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
      leftVolts *= batteryVoltage / 12.0;
      rightVolts *= batteryVoltage / 12.0;
    }
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void startDriveStraight() {
    resetGyro();
    m_gyroAngle = m_gyro.getAngle();
    controller.setTolerance(1, 5);
  }
  public void endDriveStraight() {
    m_gyroAngle = Constants.GYRO_NOTUSED;
  }

  public void startAimRedBall() {
    m_camera.setLED(VisionLEDMode.kOn);
    m_camera.setPipelineIndex(0); // This is the first pipeline from http://gloworm.local:5800
    yaw = 0;
    m_AimBall = true;
    controller.setTolerance(1, 5);
  }
  public void startAimBlueBall() {
    m_camera.setLED(VisionLEDMode.kOn);
    m_camera.setPipelineIndex(1); // This is the second pipeline from http://gloworm.local:5800
    yaw = 0;
    m_AimBall = true;
    controller.setTolerance(1, 5);
  }
  public void endAimBall() {
    m_camera.setLED(VisionLEDMode.kOff);
    m_AimBall = false;
  }
  public void startLedOn() {
    m_camera.setLED(VisionLEDMode.kOn);
  }
  public void startLedOff() {
    m_camera.setLED(VisionLEDMode.kOff);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }
  public double getAverageDistanceInch() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }
  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getGyroAngle() {
    return m_gyro.getAngle();
  }
  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private double getRightUltrasonic() {
    voltageScaleFactor = 5/RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at
    //Get a reading from the first sensor, scale it by the voltageScaleFactor, and then scale to Inches
    ultrasonicSensorRange = rightUltrasonic.getValue()*voltageScaleFactor*0.0492;
    if (ultrasonicSensorRange < 18) {
      ultrasonicSensorRange = 195;
    }
    return ultrasonicSensorRange;
  }

  private double getLeftUltrasonic() {
    voltageScaleFactor = 5/RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at
    //Get a reading from the first sensor, scale it by the voltageScaleFactor, and then scale to Inches
    ultrasonicSensorRange = leftUltrasonic.getValue()*voltageScaleFactor*0.0492;
    if (ultrasonicSensorRange < 18) {
      ultrasonicSensorRange = 195;
    }
    return ultrasonicSensorRange;
  }

  public double getUltrasonic() {
    return Math.min(getLeftUltrasonic(), getRightUltrasonic());
  }
}
