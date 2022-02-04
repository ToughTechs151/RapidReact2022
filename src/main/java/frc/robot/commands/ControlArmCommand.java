// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.oi.CoDriverOI;
import frc.robot.subsystems.ArmSubsystem;

public class ControlArmCommand extends CommandBase {
  private ArmSubsystem armSubsystem_ = null;
  private RobotContainer robotContainer_ = null;
  private CoDriverOI coDriveOI_ = null;

  /** Creates a new ControlArmCommand. */
  public ControlArmCommand(RobotContainer robotcontainer, CoDriverOI CoDriverOI) {
    robotContainer_ = robotcontainer;
    coDriveOI_ = CoDriverOI;
    addRequirements(robotcontainer.getArmSubsystem()); // here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
