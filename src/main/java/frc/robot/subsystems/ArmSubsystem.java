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
  private RelativeEncoder armEncoder = armMotor.getEncoder(); 

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    super(new PIDController(Constants.ARM_KP, Constants.ARM_KI, Constants.ARM_KD));
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(false);
    armMotor.setIdleMode(IdleMode.kBrake);
    resetEncoder();
    pid = getController();
    pid.setTolerance(Constants.POSTOLERANCE);
  }

  public void periodic() {
    // setSetpoint(RobotContainer.coDriverOI.getY()); NEVER EVER DO THIS
    super.periodic();

    SmartDashboard.putNumber("ARM Speed in RPM", armEncoder.getVelocity());
    SmartDashboard.putNumber("ARM Current", armMotor.getOutputCurrent());
    SmartDashboard.putNumber("ARMSetpoint in RPM", getSetpoint());
    SmartDashboard.putNumber("ARM get", armMotor.get());
    SmartDashboard.putNumber("ARM getCPR", armEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber("ARM getPosition", armEncoder.getPosition());
    SmartDashboard.putNumber("ARM getVelocityConversionFactor", armEncoder.getVelocityConversionFactor());
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    SmartDashboard.putNumber("ARM output value", output);
    armMotor.set(output);
  }

  @Override
  protected double getMeasurement() {
    return armEncoder.getPosition();
  }

  public void resetEncoder() {
    armEncoder.setPosition(Constants.STOP);
    setSetpoint(Constants.STOP);
  }

  public void armSetpoint(double armSetpoint){
    setSetpoint(armSetpoint);
    enable();
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }
}
