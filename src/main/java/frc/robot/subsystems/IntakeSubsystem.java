// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase { 
  WPI_TalonSRX intakemotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
  boolean isStop;
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double dir) {
    isStop = false;
    intakemotor.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED * dir);
  }

  public void stopIntake() {
    isStop = true;
    intakemotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isStop() {
    return isStop;
  }
}

