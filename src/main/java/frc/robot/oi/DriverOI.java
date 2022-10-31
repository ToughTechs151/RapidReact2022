/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ChassisSubsystem;

/** Driver OI Controls. */
public class DriverOI extends OI {

  /** Create the driver controller. */
  public DriverOI(int channel, RobotContainer robotContainer) {
    super(channel);

    abutton = new JoystickButton(joystick, Constants.A);
    ybutton = new JoystickButton(joystick, Constants.Y);
    bbutton = new JoystickButton(joystick, Constants.B);
    xbutton = new JoystickButton(joystick, Constants.X);

    ChassisSubsystem chassisSubsystem = robotContainer.getChassisSubsystem();
    leftBumper = new JoystickButton(joystick, Constants.LEFT_BUMPER);
    leftBumper.whenPressed(
        new InstantCommand(chassisSubsystem::startDriveStraight, chassisSubsystem));
    leftBumper.whenReleased(
        new InstantCommand(chassisSubsystem::endDriveStraight, chassisSubsystem));

    bbutton.whenPressed(new InstantCommand(chassisSubsystem::startAimRedBall, chassisSubsystem));
    bbutton.whenReleased(new InstantCommand(chassisSubsystem::endAimBall, chassisSubsystem));
    xbutton.whenPressed(new InstantCommand(chassisSubsystem::startAimBlueBall, chassisSubsystem));
    xbutton.whenReleased(new InstantCommand(chassisSubsystem::endAimBall, chassisSubsystem));
    abutton.whenPressed(new InstantCommand(chassisSubsystem::startLedOn, chassisSubsystem));
    ybutton.whenPressed(new InstantCommand(chassisSubsystem::startLedOff, chassisSubsystem));
  }
}
