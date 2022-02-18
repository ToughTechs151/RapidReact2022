// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder armEncoder = armMotor.getEncoder();
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  double m_armSetpoint;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(false);
    armMotor.setIdleMode(IdleMode.kBrake);
    armEncoder.setPosition(Constants.ARM_UP);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = armMotor.getPIDController();

    // PID coefficients
    //kP = 0.1;
    //kI = 1e-4;
    //kD = 1;
    kP = 0.05;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 0.5;
    kMinOutput = -0.5;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController.setReference(Constants.ARM_UP, ControlType.kPosition);
  }

  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("ARM Speed in RPM", armEncoder.getVelocity());
    SmartDashboard.putNumber("ARM Current", armMotor.getOutputCurrent());
    SmartDashboard.putNumber("ARM get", armMotor.get());
    SmartDashboard.putNumber("ARM getCPR", armEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber("ARM getPosition", armEncoder.getPosition());
    SmartDashboard.putNumber("ARM getVelocityConversionFactor", armEncoder.getVelocityConversionFactor());
  }

  public void armSetpoint(double armSetpoint){
    m_armSetpoint = armSetpoint;
    m_pidController.setReference(armSetpoint, ControlType.kPosition);
  }

  public boolean atSetpoint() {
    return armEncoder.getPosition() == m_armSetpoint;
  }
}
