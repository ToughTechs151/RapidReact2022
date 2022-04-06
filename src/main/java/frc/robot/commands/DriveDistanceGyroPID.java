// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ChassisSubsystem;

public class DriveDistanceGyroPID extends CommandBase {
  private final ChassisSubsystem drive;
  private final double distance;
  private final double speed;
  private final PIDController controller =
      new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistanceGyroPID(double speed, double inches, ChassisSubsystem drive) {
    distance = inches;
    this.speed = speed;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    // drive.tankDrive(0, 0);
    drive.resetEncoders();
    drive.resetGyro();
    // Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
    controller.setTolerance(1, 5);
    System.out.println("DriveDistanceGyroPID start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pidOutput = controller.calculate(drive.getGyroAngle(), 0) / 10;

    // smartdashboard
    // SmartDashboard.putNumber("DriveStraightPID", pidOutput);
    // SmartDashboard.putNumber("left", speed+pidOutput);
    // SmartDashboard.putNumber("right", speed-pidOutput);
    drive.tankDrive(speed + pidOutput, speed - pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
    controller.close();
    System.out.println("DriveDistanceGyroPID end " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(drive.getAverageDistanceInch()) >= distance;
  }
}
