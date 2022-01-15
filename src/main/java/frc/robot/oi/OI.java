package frc.robot.oi;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {
	Joystick joystick = null;

	protected JoystickButton x;
	protected JoystickButton a;
	protected JoystickButton b;
	protected JoystickButton y;
	protected JoystickButton leftBumper;
	protected JoystickButton rightBumper;
	protected JoystickButton leftTrigger;
	protected JoystickButton rightTrigger;
	protected JoystickButton back;
	protected JoystickButton start;
	protected JoystickButton leftJoystick;
	protected JoystickButton rightJoystick;

	public OI(int joystickChannel) {		
		joystick = new Joystick(joystickChannel);	
		x = null; 
		a = null;
		b = null;
		y = null;
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