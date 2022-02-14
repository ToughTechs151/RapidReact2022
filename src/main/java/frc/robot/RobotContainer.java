// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithJoystickCommand;
import frc.robot.oi.CoDriverOI;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static PIDController armSubsystemPID;
  public enum DriveTrainType {
    TANK,
    ARCADE,
    DEFAULT
  }

    // The robot's subsystems and commands are defined here...
  private ChassisSubsystem chassisSubsystem;
  private ArmSubsystem armSubsystem;
  private DriverOI driverOI;
  private CoDriverOI codriverOI;
  private IntakeSubsystem intakeSubsystem;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    chassisSubsystem = new ChassisSubsystem();
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
    // create driver & codriver oi
    driverOI = new DriverOI(Constants.DRIVER_OI, this);
    codriverOI = new CoDriverOI(Constants.CODRIVE_OI, this);

    // drive with joysticks
    chassisSubsystem.setDefaultCommand(new DriveWithJoystickCommand(this, driverOI));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  /**
   * retrieves chassis subsystem
   * @return
   */
  public ChassisSubsystem getChassisSubsystem() { 
    return chassisSubsystem; 
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

}
