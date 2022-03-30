// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonomousLeftDump2;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.AutonomousRightRightDump2;
import frc.robot.commands.AutonomousCenterDump;
import frc.robot.commands.AutonomousNothing;
import frc.robot.commands.AutonomousRightLeftDump2;
import frc.robot.commands.AutonomousLeftDump;
import frc.robot.commands.AutonomousTaxi;
import frc.robot.commands.AutonomousTurn90;
import frc.robot.commands.DriveWithJoystickCommand;
import frc.robot.oi.CoDriverOI;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static PIDController armSubsystemPID;
  public enum DriveTrainType {
    TANK("Tank"),
    ARCADE("Arcade");

    private String type;

    DriveTrainType(String type) {
      this.type = type;
    }

    public String getType() {
      return type;
    }
  }

    // The robot's subsystems and commands are defined here...
  private DriveSubsystem m_robotDrive;
  private ArmSubsystem armSubsystem;
  private DriverOI driverOI;
  private CoDriverOI codriverOI;
  private IntakeSubsystem intakeSubsystem;
  private DriveTrainType driveTrainType = DriveTrainType.TANK;
  private PhotonCamera m_camera = new PhotonCamera("gloworm");
  private SendableChooser<Command> chooser = new SendableChooser<>();

  private AutonomousNothing autonomousNothing;
  private AutonomousTaxi autonomousTaxi;

  private AutonomousLeftDump autonomousLeftDump;
  private AutonomousLeftDump2 autonomousLeftDump2;

  private AutonomousCenterDump autonomousCenterDump;

  private AutonomousRightLeftDump2 autonomousRightLeftDump2;
  private AutonomousRightRightDump2 autonomousRightRightDump2;

  private AutonomousTurn90 autonomousTurn90;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_robotDrive = new DriveSubsystem();
    armSubsystem = new ArmSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    // Configure the button bindings for all buttons
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // configure the autonomonous chooser

    autonomousNothing = new AutonomousNothing(this);
    autonomousTaxi = new AutonomousTaxi(this);

    autonomousLeftDump = new AutonomousLeftDump(this);
    autonomousLeftDump2 = new AutonomousLeftDump2(this);

    autonomousCenterDump = new AutonomousCenterDump(this);

    autonomousRightLeftDump2 = new AutonomousRightLeftDump2(this);

    autonomousRightRightDump2 = new AutonomousRightRightDump2(this);

    autonomousTurn90 = new AutonomousTurn90(this);

    chooser.addOption("No Auto", autonomousNothing);
    chooser.setDefaultOption("Taxi", autonomousTaxi);

    chooser.addOption("Left Dump", autonomousLeftDump);
    chooser.addOption("Left Dump 2", autonomousLeftDump2);

    chooser.addOption("Center Dump", autonomousCenterDump);

    chooser.addOption("Right Left Dump 2", autonomousRightLeftDump2);

    chooser.addOption("Right Right Dump 2", autonomousRightRightDump2);

    chooser.addOption("Turn 90", autonomousTurn90);

    // Put the chooser on the dashboard
    SmartDashboard.putData(chooser);

    // create driver & codriver oi
    driverOI = new DriverOI(Constants.DRIVER_OI, this);
    codriverOI = new CoDriverOI(Constants.CODRIVER_OI, this);

    // drive with joysticks
    m_robotDrive.setDefaultCommand(new DriveWithJoystickCommand(this, driverOI));
    m_camera.setLED(VisionLEDMode.kOff);
  }

  /**
   * set the drive train type
   * @param type
   */
  public void setDriveTrainType(String type) {
    // SmartDashboard.putString(Constants.DRIVE_TRAIN_TYPE, type);
    if (type.equals(Constants.TANK)) {
      driveTrainType = DriveTrainType.TANK;
    } else if (type.equals(Constants.ARCADE)) {
      driveTrainType = DriveTrainType.ARCADE;
    }
  }

  /**
   * getDriveTrainType
   * @return returns the DriveTrainType - Tank Drive or Arcade Drive
   */
  public DriveTrainType getDriveTrainType() {
    return driveTrainType;
  }

  public DriveSubsystem getRobotDrive() {
    return m_robotDrive;
  }

  /** Zeros the outputs of all subsystems. */
  public void zeroAllOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return chooser.getSelected();
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            7);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at (1, 2) facing the +X direction
            new Pose2d(1, 2, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4, 2, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(
                Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to starting pose of trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

  /**
   * retrieves the arm subsystem
   *
   * @return
   */
  public ArmSubsystem getArmSubsystem() {
    return armSubsystem;
  }

  /**
   * retrives the IntakeSubsystem
   * @return
   */
  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public PhotonCamera getCamera() {
    return m_camera;
  }

}
