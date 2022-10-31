/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ControlArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.StopIntakeCommand;

/** CoDriver OI Controls. */
public class CoDriverOI extends OI {
  /** Create the co-driver controller object. */
  public CoDriverOI(int channel, RobotContainer robotContainer) {
    super(channel);

    rightBumper = new JoystickButton(joystick, Constants.RIGHT_BUMPER);
    rightBumper.whileHeld(new IntakeCommand(robotContainer, Constants.INTAKE_OUT));
    rightBumper.whenReleased(new StopIntakeCommand(robotContainer));

    leftBumper = new JoystickButton(joystick, Constants.LEFT_BUMPER);
    leftBumper.whileHeld(new IntakeCommand(robotContainer, Constants.INTAKE_IN));
    leftBumper.whenReleased(new StopIntakeCommand(robotContainer));

    // ARM Control Up
    abutton = new JoystickButton(joystick, Constants.A);
    abutton.whenPressed(new ControlArmCommand(robotContainer, Constants.ARM_UP));

    // ARM Control down
    bbutton = new JoystickButton(joystick, Constants.B);
    bbutton.whenPressed(new ControlArmCommand(robotContainer, Constants.ARM_DOWN));

    // ARM Control down
    xbutton = new JoystickButton(joystick, Constants.X);
    xbutton.whenPressed(new ControlArmCommand(robotContainer));

    leftJoystick = new JoystickButton(joystick, Constants.LEFT_JOYSTICK);
    rightJoystick = new JoystickButton(joystick, Constants.RIGHT_JOYSTICK);
  }
}
