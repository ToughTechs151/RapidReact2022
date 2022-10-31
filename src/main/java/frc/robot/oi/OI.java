package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {
  Joystick joystick;

  protected JoystickButton xbutton;
  protected JoystickButton abutton;
  protected JoystickButton bbutton;
  protected JoystickButton ybutton;
  protected JoystickButton leftBumper;
  protected JoystickButton rightBumper;
  protected JoystickButton leftTrigger;
  protected JoystickButton rightTrigger;
  protected JoystickButton back;
  protected JoystickButton start;
  protected JoystickButton leftJoystick;
  protected JoystickButton rightJoystick;

  /** Create controllers. */
  public OI(int joystickChannel) {
    joystick = new Joystick(joystickChannel);
    xbutton = null;
    abutton = null;
    bbutton = null;
    ybutton = null;
    leftBumper = null;
    rightBumper = null;
    leftTrigger = null;
    rightTrigger = null;
    back = null;
    start = null;
    leftJoystick = null;
    rightJoystick = null;
  }

  public Joystick getJoystick() {
    return joystick;
  }
}
