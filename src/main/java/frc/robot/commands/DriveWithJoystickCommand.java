// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithJoystickCommand extends CommandBase {
  private DriveSubsystem m_drive;
  private DriverOI driverOI;
  private RobotContainer robotContainer;


  public enum RobotSide {
    LEFT, RIGHT;
  }

  /**
   * The scale factor for normal mode
   */
  private static final double normal = 1.0;

  /**
   * The scale factor for crawl mode
   */
  private static final double crawl = 0.5;

  /**
   * The direction which is "forward"; 1 represents the hatch side and -1 represents the cargo side.
   */
  private int dir = Constants.INTAKE_IN;


  /** Creates a new DriveWithJoystickCommand. */
  public DriveWithJoystickCommand(RobotContainer robotContainer, DriverOI driverOI) {
    this.robotContainer = robotContainer;
    this.driverOI = driverOI;
    m_drive = robotContainer.getRobotDrive();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveWithJoystick start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMultiplier = driverOI.getJoystick().getRawButton(Constants.RIGHT_BUMPER) ? crawl : normal;
    dir = driverOI.getJoystick().getRawButton(Constants.LEFT_BUMPER) ? Constants.INTAKE_OUT : Constants.INTAKE_IN;

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
      m_drive.tankDrive(leftVal * speedMultiplier * dir, rightVal * speedMultiplier * dir);
    } else if (driveTrainType == RobotContainer.DriveTrainType.ARCADE) {
      double speed = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y) * dir;
      double rotation = driverOI.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_X);
      SmartDashboard.putNumber("Speed", speed);
      SmartDashboard.putNumber("Rotation", rotation);
      m_drive.arcadeDrive(speed, rotation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveWithJoystickCommand end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
