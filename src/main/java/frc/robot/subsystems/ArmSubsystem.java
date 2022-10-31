// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController smPidController;
  private RelativeEncoder armEncoder = armMotor.getEncoder();
  public double konP;
  public double konI;
  public double konD;
  public double konIz;
  public double konFF;
  public double konMaxOutput;
  public double konMinOutput;
  public double maxVel;
  public double minVel;
  public double maxAcc;
  double armSetpoint;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(false);
    armMotor.setIdleMode(IdleMode.kBrake);
    armEncoder.setPosition(0);
    SmartDashboard.putNumber("ARM Target", 0.0);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object is
     * constructed by calling the getPIDController() method on an existing CANSparkMax object
     */
    smPidController = armMotor.getPIDController();

    // The following is based on SparkMax Smart Motion Example
    // PID coefficients
    konP = 5e-5;
    konI = 1e-6;
    konD = 0;
    konIz = 0;
    konFF = 0.000156;
    konMaxOutput = 10;
    konMinOutput = -0.5;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1000;

    // set PID coefficients
    smPidController.setP(konP);
    smPidController.setI(konI);
    smPidController.setD(konD);
    smPidController.setIZone(konIz);
    smPidController.setFF(konFF);
    smPidController.setOutputRange(konMinOutput, konMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object.
     *
     * <p>- setSmartMotionMaxVelocity() will limit the velocity in RPM of the pid controller in
     * Smart Motion mode - setSmartMotionMinOutputVelocity() will put a lower bound in RPM of the
     * pid controller in Smart Motion mode - setSmartMotionMaxAccel() will limit the acceleration in
     * RPM^2 of the pid controller in Smart Motion mode - setSmartMotionAllowedClosedLoopError()
     * will set the max allowed error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    smPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    smPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    smPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
  }

  /** Run subsystem code periodically. */
  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("ARM Speed in RPM", armEncoder.getVelocity());
    SmartDashboard.putNumber("ARM Current", armMotor.getOutputCurrent());
    SmartDashboard.putNumber("ARM get", armMotor.get());
    SmartDashboard.putNumber("ARM getCPR", armEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber("Arm position", armEncoder.getPosition());
    SmartDashboard.putNumber(
        "ARM getVelocityConversionFactor", armEncoder.getVelocityConversionFactor());
    SmartDashboard.putNumber("ARM BusVoltage", armMotor.getBusVoltage());
    SmartDashboard.putNumber("ARM Applied", armMotor.getAppliedOutput());
    SmartDashboard.putNumber("ARM Temp", armMotor.getMotorTemperature());
    SmartDashboard.putNumber("kP", konP);
    SmartDashboard.putNumber("kI", konI);
    SmartDashboard.putNumber("kD", konD);
    SmartDashboard.putNumber("kIz", konIz);
    SmartDashboard.putNumber("kFF", konFF);
    SmartDashboard.putNumber("kMaxOutput", konMaxOutput);
    SmartDashboard.putNumber("kMinOutput", konMinOutput);
    if (armMotor.getIdleMode() == IdleMode.kCoast) {
      SmartDashboard.putString("ARM Idle Mode", "Coast");
    } else {
      SmartDashboard.putString("ARM Idle Mode", "Brake");
    }
  }

  public void armSetpoint(double armSetpoint) {
    this.armSetpoint = armSetpoint;
    smPidController.setReference(armSetpoint, ControlType.kSmartMotion);
  }

  public boolean atSetpoint() {
    return armEncoder.getPosition() == armSetpoint;
  }
}
