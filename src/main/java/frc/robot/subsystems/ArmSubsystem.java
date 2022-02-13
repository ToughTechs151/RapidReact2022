// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class ArmSubsystem extends PIDSubsystem {
    // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private PIDController pid;

  private CANSparkMax armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
  /** Creates a new ArmSubsystem. */
  private RelativeEncoder armEncoder = armMotor.getEncoder(); 

  public ArmSubsystem() {
    super(new PIDController(Constants.ARMKP, Constants.ARMKI, Constants.ARMKD));
    armMotor.setInverted(false);
    armMotor.setIdleMode(IdleMode.kBrake);
    mySetSetpoint(50);
  }

   /**
   * runs periodically when enabled
  /**
   * change the ARM position until it hits the limit switch and hold the position
   * @param position
   */
  public void changePosition(int position) {
    if (position == Constants.UP) {
    } else if (position == Constants.DOWN) {

    }
  }

  public void periodic() {
    // setSetpoint(RobotContainer.coDriverOI.getY()); NEVER EVER DO THIS
    super.periodic();
    //useOutput(armEncoder.getVelocity(), setpoint);
    SmartDashboard.putNumber("UpperLauncherSpeed in RPM", armEncoder.getVelocity());
    SmartDashboard.putNumber("UpperLauncher Current", armMotor.getOutputCurrent());
    SmartDashboard.putNumber("UpperLauncherSetpoint in RPM", getSetpoint());
    SmartDashboard.putNumber("UpperLauncher get", armMotor.get());
    SmartDashboard.putNumber("UpperLauncher getCPR", armEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber("UpperLauncher getPosition", armEncoder.getPosition());
    SmartDashboard.putNumber("UpperLauncher getVelocityConversionFactor", armEncoder.getVelocityConversionFactor());
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    armMotor.set(output);
    double viewOutput = output;
    SmartDashboard.putNumber("Launcher output value", viewOutput);
    /*if (Math.abs(setpoint) >= Math.abs(getController().getSetpoint())) {
      setpoint = (getController().getSetpoint() * 10.0 / 9.0);
      if (setpoint >= this.setpoint - 100 && setpoint <= this.setpoint + 100) {
        setpoint = this.setpoint;
      }
    } else if (Math.abs(setpoint) < Math.abs(getController().getSetpoint())) {
      setpoint = (getController().getSetpoint() * 0.9);
      if (setpoint >= this.setpoint - 100 && setpoint <= this.setpoint + 100) {
        setpoint=this.setpoint;
      }
    }
    output=getController().calculate(output,setpoint)/armEncoder.getVelocityConversionFactor();
    launcher1.set(output);*/
    //launcher1.set(output*0.95);    
    //System.out.println("UpperLauncher:" + output*0.95);
    //launcher2.set(output); 
}

@Override
protected double getMeasurement() {
  return armEncoder.getVelocity();
}

public void mySetSetpoint(double mySetpoint){
  m_enabled = true;
  setSetpoint(mySetpoint);
}

public void disableM_Enabled(){
  setSetpoint(0);
  m_enabled = false;
}

/**
 * sets the launcher speed
 * @param s the speed to change to
 */
/*public static void setSpeed(double s){
  launcher1.set(s/armEncoder.getVelocityConversionFactor());

}*/
}
