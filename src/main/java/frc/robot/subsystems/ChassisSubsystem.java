// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.oi.DriverOI;

public class ChassisSubsystem extends SubsystemBase {
  public enum RobotSide {
    LEFT, RIGHT;
  }

  private Talon frontLeftMotor_ = new Talon(Constants.FRONT_LEFT_MOTOR);
  private Talon frontRightMotor_ = new Talon(Constants.FRONT_RIGHT_MOTOR);
  private Talon rearLeftMotor_ = new Talon(Constants.REAR_LEFT_MOTOR);
  private Talon rearRightMotor_ = new Talon(Constants.REAR_RIGHT_MOTOR);

  private MotorControllerGroup leftMotors_ = new MotorControllerGroup(frontLeftMotor_, rearLeftMotor_);
  private MotorControllerGroup rightMotors_ = new MotorControllerGroup(frontRightMotor_, rearRightMotor_);

  private DifferentialDrive driveTrain_ = new DifferentialDrive(leftMotors_, rightMotors_);
  /** Creates a new ChassisSubsystem. */
  public ChassisSubsystem() {
    rightMotors_.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /**
   * 
   * @param leftspeed - The left joystick controller spped -1 to 1
   * @param rightspeed - The right joystick controller speed -1 to 1
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain_.tankDrive(leftSpeed, rightSpeed);
  }

  public void drive(DriverOI driverOI, int scale) {
    
  }
}
