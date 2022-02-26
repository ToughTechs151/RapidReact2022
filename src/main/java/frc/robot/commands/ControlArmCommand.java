// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class ControlArmCommand extends CommandBase {
  private ArmSubsystem armSubsystem;
  private double setPoint;

  /**
   * ControlArmCommand - commanding the arm to move up or down and hold its position when the arm
   * reach the final position
   *
   * @param robotcontainer
   * @param position
   */
  public ControlArmCommand(RobotContainer robotcontainer, int position) {
    armSubsystem = robotcontainer.getArmSubsystem();
    addRequirements(armSubsystem); // here to declare subsystem dependencies.
    setPoint = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.armSetpoint(setPoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atSetpoint();
  }
}
