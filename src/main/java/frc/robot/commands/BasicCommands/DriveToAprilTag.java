// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class DriveToAprilTag extends Command {

  private final CommandSwerveDrivetrain swerveDrive;
  private final double desiredDistance = 0.301; // Meters from tag (Z-axis)
  private final double desiredOffset = .187;   // Meters left/right (X-axis)
  private final PIDController xController; // Lateral offset
  private final PIDController yController; // Distance
  private final SwerveRequest.FieldCentricFacingAngle driveRequest;

  public DriveToAprilTag(CommandSwerveDrivetrain swerveDrive) {
    this.swerveDrive = swerveDrive;

    // Tune these PID values based on your robot!
    this.xController = new PIDController(2, 0.0, 0.0); // Lateral PID
    this.yController = new PIDController(2, 0.0, 0.0); // Distance PID

    this.driveRequest = new SwerveRequest.FieldCentricFacingAngle();

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    // Reset PID controllers
    xController.reset();
    yController.reset();
  }

  @Override
  public void execute()
  {
    // Get robot's pose relative to the tag
    var pose2 = LimelightHelpers.getBotPose3d_TargetSpace(Constants.Limelight.RIGHT);
    SmartDashboard.putNumber("TargetPoseleft X", pose2.getX());
    SmartDashboard.putNumber("TargetPoseleft Y", pose2.getY());
    boolean tv = LimelightHelpers.getTV(LimelightHelpers.getWhichLimelight()); // Target validity

    if (!tv) {
        // No valid target or data, stop the robot
        swerveDrive.setControl(driveRequest.withVelocityX(0).withVelocityY(0));
        return;
    }

    // Current pose in tag's frame
    double currentX = pose2.getX(); // Lateral offset from tag (meters)
    double currentY = pose2.getY(); // Distance from tag (meters)


    // Calculate errors
    double xError = desiredOffset - currentX; // Positive = move right in tag frame
    double yError = desiredDistance - currentY; // Positive = move forward in tag frame

    // Calculate PID outputs
    double xSpeed = xController.calculate(xError); // Meters per second (Y in field frame)
    double ySpeed = yController.calculate(yError); // Meters per second (X in field frame)

    // Limit speeds
    xSpeed = Math.max(-1.0, Math.min(1.0, xSpeed));
    ySpeed = Math.max(-1.0, Math.min(1.0, ySpeed));

    // Apply field-centric control
    swerveDrive.setControl(driveRequest
        .withVelocityX(-xSpeed) // Forward/backward
        .withVelocityY(-ySpeed) // Left/right
        .withTargetDirection(Rotation2d.fromDegrees(180))); // Rotation
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
        // Get robot's pose relative to the tag
        var pose2 = LimelightHelpers.getBotPose3d_TargetSpace(Constants.Limelight.RIGHT);
        SmartDashboard.putNumber("TargetPoseleft X", pose2.getX());
        SmartDashboard.putNumber("TargetPoseleft Y", pose2.getY());
        boolean tv = LimelightHelpers.getTV(LimelightHelpers.getWhichLimelight()); // Target validity
    if (!tv) return false;

    double currentX = pose2.getX();
    double currentY = pose2.getY();

    double xError = Math.abs(desiredOffset - currentX);
    double zError = Math.abs(desiredDistance - currentY);

    return xError < 0.05 && zError < 0.05 ; // 5 cm, 2 degrees tolerance
  }
}
