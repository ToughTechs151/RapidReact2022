// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ChassisSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveDistanceGyroPID extends CommandBase {
  private final ChassisSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;
  private final PIDController m_controller = new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistanceGyroPID(double speed, double inches, ChassisSubsystem drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
    // Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
    m_controller.setTolerance(1, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double error = -m_drive.getGyroAngleZ();
    //double turn_power = Constants.DIVIDER  * error;
    
    var pidOutput = 
    
    m_controller.calculate(m_drive.getGyroAngle(), m_distance);

    // Clamps the controller output to between -0.5 and 0.5
    pidOutput = MathUtil.clamp(pidOutput, -m_speed, m_speed);

    //smartdashboard
    SmartDashboard.putNumber("DriveStraightPID", pidOutput);
    m_drive.arcadeDrive(m_speed, pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}