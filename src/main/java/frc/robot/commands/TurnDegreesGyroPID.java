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

public class TurnDegreesGyroPID extends CommandBase {
  private final ChassisSubsystem m_drive;
  private final double m_degrees;
  private final double m_speed;
  private final PIDController m_controller = new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);


  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegreesGyroPID(double speed, double degrees, ChassisSubsystem drive) {
    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_drive.resetGyro();
    // Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
    m_controller.setTolerance(1, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pidOutput = m_controller.calculate(m_drive.getGyroAngle(), m_degrees);
    // Clamps the controller output output between -0.5 and 0.5
    pidOutput = MathUtil.clamp(pidOutput, -m_speed, m_speed);
    SmartDashboard.putNumber("TurnPID", pidOutput);
    SmartDashboard.putNumber("GyroValue", m_drive.getGyroAngle());
    m_drive.arcadeDrive(0, pidOutput); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return m_controller.atSetpoint();
  }
}
