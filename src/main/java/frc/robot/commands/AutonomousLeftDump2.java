// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousLeftDump2 extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousLeftDump2(RobotContainer robotContainer) {
    DriveSubsystem m_robotDrive = robotContainer.getRobotDrive();
    
    addCommands( 

    new ControlArmCommand(robotContainer, Constants.ARM_DOWN),
    new IntakeCommand(robotContainer, Constants.INTAKE_IN),
    new DriveDistanceGyroPID(0.4, 50, m_robotDrive),
    new WaitCommand(1),
    new ControlArmCommand(robotContainer, Constants.ARM_UP),
    new StopIntakeCommand(robotContainer),
    new TurnDegreesGyroPID(0.6, 180, m_robotDrive),
    new DriveDistanceGyroPID(0.5, 50, m_robotDrive),
    new DriveDistanceGyroUltrasonic(0.4, 23, m_robotDrive),
    new IntakeCommand(robotContainer, Constants.INTAKE_OUT),
    new WaitCommand(1),
    new StopIntakeCommand(robotContainer),
    new DriveDistanceGyroPID(-0.5, 80, m_robotDrive)
    
    );
    
  } 
}
