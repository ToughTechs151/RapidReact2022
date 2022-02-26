// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.ChassisSubsystem;

public class DriveWithJoystickCommand extends CommandBase {
  private ChassisSubsystem chassisSubsystem;
  private DriverOI driverOI;
  private RobotContainer robotContainer;

  /** Creates a new DriveWithJoystickCommand. */
  public DriveWithJoystickCommand(RobotContainer robotContainer, DriverOI driverOI) {
    this.robotContainer = robotContainer;
    this.driverOI = driverOI;
    chassisSubsystem = this.robotContainer.getChassisSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSubsystem.drive(robotContainer, driverOI, Constants.SCALED_DRIVE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
