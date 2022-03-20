// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ChassisSubsystem;

public class AutonomousCenterLeft extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousCenterLeft(RobotContainer robotContainer) {
    ChassisSubsystem chassisSubsystem = robotContainer.getChassisSubsystem();
    addCommands( 
    new DriveDistanceGyroPID(0.5, 50, chassisSubsystem),
    new TurnDegreesGyroPID(0.5, 90, chassisSubsystem),
    new ControlArmCommand(robotContainer, Constants.ARM_DOWN),
    new IntakeCommand(robotContainer, Constants.INTAKE_IN),
    new DriveDistanceGyroPID(0.5, 30, chassisSubsystem),
    new StopIntakeCommand(robotContainer),
    new ControlArmCommand(robotContainer, Constants.ARM_UP),
    new TurnDegreesGyroPID(0.5, 180, chassisSubsystem),
    new DriveDistanceGyroPID(0.5, 10, chassisSubsystem),
    new TurnDegreesGyroPID(-0.5, 90, chassisSubsystem),
    new DriveDistanceGyroPID(0.5, 100, chassisSubsystem),
    new IntakeCommand(robotContainer, Constants.INTAKE_OUT),
    new WaitCommand(1),
    new StopIntakeCommand(robotContainer)
    );
  } 
}