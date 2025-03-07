// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Helpers.LimelightHelpers;

public class DriveToAprilTag extends Command {

  private final CommandSwerveDrivetrain swerveDrive;
  private final double desiredDistance = 0.5; // Meters from tag (Z-axis)
  private final double desiredOffset = 0;   // Meters left/right (X-axis)
  private final double desiredAngle = 0;    // Degrees relative to tag
  private final PIDController xController; // Lateral offset
  private final PIDController zController; // Distance
  private final PIDController angleController; // Yaw
  private final SwerveRequest.FieldCentric driveRequest;
  private final String limelight;

  public DriveToAprilTag(CommandSwerveDrivetrain swerveDrive, String limelight) {
    this.swerveDrive = swerveDrive;
    this.limelight = limelight;

    // Tune these PID values based on your robot!
    this.xController = new PIDController(2, 0.0, 0.0); // Lateral PID
    this.zController = new PIDController(2, 0.0, 0.0); // Distance PID
    this.angleController = new PIDController(2, 0.0, 0.0); // Angle PID

    this.driveRequest = new SwerveRequest.FieldCentric();
    this.angleController.enableContinuousInput(-180.0, 180.0);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    // Reset PID controllers
    xController.reset();
    zController.reset();
    angleController.reset();
  }

  @Override
  public void execute()
  {
    // Get robot's pose relative to the tag
    double[] botPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");
    boolean tv = LimelightHelpers.getTV(limelight); // Target validity

    if (!tv) {
        // No valid target or data, stop the robot
        swerveDrive.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        return;
    }

    // Current pose in tag's frame
    double currentX = botPoseTargetSpace[0]; // Lateral offset from tag (meters)
    double currentZ = botPoseTargetSpace[2]; // Distance from tag (meters)
    double tagRelativeYaw = botPoseTargetSpace[5]; // Robot yaw relative to tag (degrees)

    // Get robot's field-relative yaw from swerve drivetrain
    double currentFieldYaw = swerveDrive.getState().Pose.getRotation().getDegrees();

    // Desired yaw: Tag's yaw (in field frame) + desired angle offset
    // tagRelativeYaw is robot yaw relative to tag, so adjust to field frame
    double desiredFieldYaw = currentFieldYaw - tagRelativeYaw + desiredAngle;

    // Calculate errors
    double xError = desiredOffset - currentX; // Positive = move right in tag frame
    double zError = desiredDistance - currentZ; // Positive = move forward in tag frame
    double angleError = desiredFieldYaw - currentFieldYaw; // Positive = rotate CCW in field frame

    // Calculate PID outputs
    double xSpeed = xController.calculate(xError); // Meters per second (Y in field frame)
    double zSpeed = zController.calculate(zError); // Meters per second (X in field frame)
    double rotSpeed = angleController.calculate(angleError); // Radians per second

    // Limit speeds
    xSpeed = Math.max(-1.0, Math.min(1.0, xSpeed));
    zSpeed = Math.max(-1.0, Math.min(1.0, zSpeed));
    rotSpeed = Math.max(-Math.PI, Math.min(Math.PI, rotSpeed));

    // Apply field-centric control
    swerveDrive.setControl(driveRequest
        .withVelocityX(zSpeed) // Forward/backward
        .withVelocityY(xSpeed) // Left/right
        .withRotationalRate(rotSpeed)); // Rotation
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    double[] botPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");
    boolean tv = LimelightHelpers.getTV(limelight); // Convert double to boolean
    if (!tv) return false;

    double currentX = botPoseTargetSpace[0];
    double currentZ = botPoseTargetSpace[2];
    double tagRelativeYaw = botPoseTargetSpace[5];
    double currentFieldYaw = swerveDrive.getState().Pose.getRotation().getDegrees();
    double desiredFieldYaw = currentFieldYaw - tagRelativeYaw + desiredAngle;

    double xError = Math.abs(desiredOffset - currentX);
    double zError = Math.abs(desiredDistance - currentZ);
    double angleError = Math.abs(desiredFieldYaw - currentFieldYaw);

    return xError < 0.05 && zError < 0.05 && angleError < 2.0; // 5 cm, 2 degrees tolerance
  }
}
