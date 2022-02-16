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

/**
 * CoDriver OI Controls
 */
public class CoDriverOI extends OI {

    public CoDriverOI(int channel, RobotContainer robotContainer) {
        super(channel);
        
        if (getJoystick().isConnected()) {
            rightBumper = new JoystickButton(joystick, Constants.RIGHT_BUMPER);
            rightBumper.whileHeld(new IntakeCommand(robotContainer, Constants.FORWARD));
            rightBumper.whenReleased(new StopIntakeCommand(robotContainer));
        
            leftBumper = new JoystickButton(joystick, Constants.LEFT_BUMPER);    
            leftBumper.whileHeld(new IntakeCommand(robotContainer, Constants.REVERSE));
            leftBumper.whenReleased(new StopIntakeCommand(robotContainer));

            // ARM Control Up
            a = new JoystickButton(joystick, Constants.A);
            a.whenPressed(new ControlArmCommand(robotContainer, Constants.UP));

            // ARM Control down
            b = new JoystickButton(joystick, Constants.B);
            b.whenPressed(new ControlArmCommand(robotContainer, Constants.DOWN));
        
            leftJoystick = new JoystickButton(joystick, Constants.LEFT_JOYSTICK);
            rightJoystick = new JoystickButton(joystick, Constants.RIGHT_JOYSTICK);
        }
    }
}