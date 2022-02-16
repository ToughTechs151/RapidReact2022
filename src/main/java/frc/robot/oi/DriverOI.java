/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Driver OI Controls
 */
public class DriverOI extends OI {

    public DriverOI(int channel, RobotContainer robotContainer) {
        super(channel);
        
        if (getJoystick().isConnected()) {
            a = new JoystickButton(joystick, Constants.A);
            y = new JoystickButton(joystick, Constants.Y);
            b = new JoystickButton(joystick, Constants.B);
            a = new JoystickButton(joystick, Constants.A);
        }
    }
}
