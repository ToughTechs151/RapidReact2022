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
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class TurnVisionAimBall extends CommandBase {
  private final ChassisSubsystem drive;
  private final double speed;
  private final PIDController controller = new PIDController(Constants.LIMELIGHT_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);
  private final PhotonCamera m_camera;
  private double yaw = 0;


  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnVisionAimBall(double speed, ChassisSubsystem drive, PhotonCamera camera) {
    this.speed = speed;
    this.drive = drive;
    this.m_camera = camera;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
    controller.setTolerance(3, 5);
    m_camera.setLED(VisionLEDMode.kOn);
    yaw = 0;
    System.out.println("TurnVisionAimBall start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      yaw = result.getBestTarget().getYaw();
      SmartDashboard.putNumber("Yaw", yaw);
    }
    var pidOutput = controller.calculate(yaw, 0);
    // Set minimum voltage for motors
    if(pidOutput < 0) {
      pidOutput = Math.min(pidOutput, -0.35);
    } else {
      pidOutput = Math.max(pidOutput, 0.35);
    }
    // Clamps the controller output output
    pidOutput = MathUtil.clamp(pidOutput, -speed, speed);
    SmartDashboard.putNumber("TurnPID", pidOutput);
    drive.arcadeDrive(0, pidOutput);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
    m_camera.setLED(VisionLEDMode.kOff);
    System.out.println("TurnVisionAimBall end " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(yaw < 2)
      return true;
    else
      return false;
  }
}
