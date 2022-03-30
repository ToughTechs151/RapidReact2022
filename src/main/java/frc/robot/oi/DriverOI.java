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
import frc.robot.subsystems.DriveSubsystem;

/**
 * Driver OI Controls
 */
public class DriverOI extends OI {

    public DriverOI(int channel, RobotContainer robotContainer) {
        super(channel);

        a = new JoystickButton(joystick, Constants.A);
        y = new JoystickButton(joystick, Constants.Y);
        b = new JoystickButton(joystick, Constants.B);
        x = new JoystickButton(joystick, Constants.X);

        DriveSubsystem m_robotDrive = robotContainer.getRobotDrive();
        leftBumper = new JoystickButton(joystick, Constants.LEFT_BUMPER);
        leftBumper.whenPressed(new InstantCommand(m_robotDrive::startDriveStraight, m_robotDrive));
        leftBumper.whenReleased(new InstantCommand(m_robotDrive::endDriveStraight, m_robotDrive));

        b.whenPressed(new InstantCommand(m_robotDrive::startAimRedBall, m_robotDrive));
        b.whenReleased(new InstantCommand(m_robotDrive::endAimBall, m_robotDrive));
        x.whenPressed(new InstantCommand(m_robotDrive::startAimBlueBall, m_robotDrive));
        x.whenReleased(new InstantCommand(m_robotDrive::endAimBall, m_robotDrive));
        a.whenPressed(new InstantCommand(m_robotDrive::startLedOn, m_robotDrive));
        y.whenPressed(new InstantCommand(m_robotDrive::startLedOff, m_robotDrive));
    }
}
