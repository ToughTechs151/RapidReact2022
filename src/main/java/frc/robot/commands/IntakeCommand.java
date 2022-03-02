// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  boolean isFinished;
  private int dir = 1;

  /** Creates a new RunIntake. */
  public IntakeCommand(RobotContainer robotContainer, int dir) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = robotContainer.getIntakeSubsystem();
    this.dir = dir;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("IntakeCommand start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("IntakeCommand execute");
    intakeSubsystem.runIntake(dir);
    if (intakeSubsystem.isStop()) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("IntakeCommand end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
