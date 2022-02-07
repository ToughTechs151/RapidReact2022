// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotContainer;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.math.controller.PIDController;

public class ArmSubsystem extends PIDSubsystem {
    // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static PIDController pid;

  private CANSparkMax armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
  /** Creates a new ArmSubsystem. */
  private static RelativeEncoder Encoder1 = armMotor_.getEncoder(); 
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  public ArmSubsystem(PIDController inPID) {
    super(inPID);
    armMotor_.setInverted(false);
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
  public void periodic() {
    // setSetpoint(RobotContainer.coDriverOI.getY()); NEVER EVER DO THIS
    super.periodic();
    //useOutput(Encoder1.getVelocity(), setpoint);
    SmartDashboard.putNumber("UpperLauncherSpeed in RPM", Encoder1.getVelocity());
    SmartDashboard.putNumber("UpperLauncher Current", armMotor_.getOutputCurrent());
    SmartDashboard.putNumber("UpperLauncherSetpoint in RPM", getSetpoint());
    SmartDashboard.putNumber("UpperLauncher get", armMotor_.get());
    SmartDashboard.putNumber("UpperLauncher getCPR", Encoder1.getCountsPerRevolution());
    SmartDashboard.putNumber("UpperLauncher getPosition", Encoder1.getPosition());
    SmartDashboard.putNumber("UpperLauncher getVelocityConversionFactor", Encoder1.getVelocityConversionFactor());
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    armMotor_.set(output);
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
    output=getController().calculate(output,setpoint)/Encoder1.getVelocityConversionFactor();
    launcher1.set(output);*/
    //launcher1.set(output*0.95);    
    //System.out.println("UpperLauncher:" + output*0.95);
    //launcher2.set(output); 
}

@Override
protected double getMeasurement() {
  return Encoder1.getVelocity();
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
  launcher1.set(s/Encoder1.getVelocityConversionFactor());

}*/
}
