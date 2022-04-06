// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ChassisSubsystem;

public class TurnDegreesGyroPID extends CommandBase {
  private final ChassisSubsystem drive;
  private final double degrees;
  private final double speed;
  private final PIDController controller =
      new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegreesGyroPID(double speed, double degrees, ChassisSubsystem drive) {
    this.speed = Math.abs(speed);
    this.degrees = degrees;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    drive.resetEncoders();
    drive.resetGyro();
    // Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
    controller.setTolerance(6, 5);
    System.out.println("TurnDegrees start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pidOutput = controller.calculate(drive.getGyroAngle(), degrees);
    // Clamps the controller output output between -0.5 and 0.5
    if (pidOutput < 0) {
      pidOutput = Math.min(pidOutput, -0.28);
    } else {
      pidOutput = Math.max(pidOutput, 0.28);
    }
    pidOutput = MathUtil.clamp(pidOutput, -speed, speed);
    // SmartDashboard.putNumber("TurnPID", pidOutput);
    // SmartDashboard.putNumber("GyroValue", drive.getGyroAngle());
    drive.arcadeDrive(0, pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    System.out.println("TurnDegreesGyroPID end " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
