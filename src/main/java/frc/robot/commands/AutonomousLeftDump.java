// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ChassisSubsystem;

public class AutonomousLeftDump extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousLeftDump(RobotContainer robotContainer) {
    ChassisSubsystem chassisSubsystem = robotContainer.getChassisSubsystem();
    addCommands(
        new DriveDistanceGyroPID(0.5, 15, chassisSubsystem),
        new TurnDegreesGyroPID(0.7, -22.5, chassisSubsystem),
        new DriveDistanceGyroUltrasonic(0.4, 23, chassisSubsystem),
        new IntakeCommand(robotContainer, Constants.INTAKE_OUT),
        new WaitCommand(1),
        new StopIntakeCommand(robotContainer),
        new DriveDistanceGyroPID(-0.5, 80, chassisSubsystem));
  }
}
