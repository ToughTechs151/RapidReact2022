// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ChassisSubsystem;

public class AutonomousTurn90 extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousTurn90(RobotContainer robotContainer) {
    ChassisSubsystem chassisSubsystem = robotContainer.getChassisSubsystem();
    addCommands(
        new TurnDegreesGyroPID(0.6, 90, chassisSubsystem),
        new TurnDegreesGyroPID(0.6, 90, chassisSubsystem),
        new TurnDegreesGyroPID(0.6, 90, chassisSubsystem),
        new TurnDegreesGyroPID(0.6, 90, chassisSubsystem));
  }
}
