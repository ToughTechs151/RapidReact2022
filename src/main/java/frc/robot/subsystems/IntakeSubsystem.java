// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase { 
  WPI_TalonSRX intakemotor_ = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
  //Error range for middle to prevent oscillations
  private double timeStarted = 0.0;
  private double timeElapsed = 4.5;
  private double currentTime;

  //Drop and raise the intake to load in balls
  public boolean deployIntake(int direction) {
    //Check if method has been called before
    if (timeStarted == 0.0) {
      timeStarted = Timer.getFPGATimestamp();
    }

    currentTime = Timer.getFPGATimestamp();
    if (currentTime - timeStarted >= timeElapsed) {
      //System.out.println("Time Elapsed is " + (currentTime-timeStarted) + " seconds");
      timeStarted = 0.0;
      intakemotor_.set(0);
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double dir_) {
      intakemotor_.set(ControlMode.PercentOutput, dir_);
  }

  public void stopIntake() {
    intakemotor_.set(ControlMode.PercentOutput, 0);
  }
}

