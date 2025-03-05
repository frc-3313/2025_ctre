// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GoToPosAndRot extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d targetPose;
  private final PIDController xController;
  private final PIDController yController;

  // Tolerance for position (meters) and heading (degrees)
  private static final double POSITION_TOLERANCE = 0.02;
  private static final double HEADING_TOLERANCE = Math.toRadians(1);

  private FieldCentricFacingAngle swerveRequest = new FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity);

  public GoToPosAndRot(CommandSwerveDrivetrain drivetrain, double xPos, double yPos, double rot) {
    this.drivetrain = drivetrain;
    this.targetPose = new Pose2d(xPos, yPos, Rotation2d.fromDegrees(rot));

    this.xController = new PIDController(4.0, 0.0001, 0.5);
    this.yController = new PIDController(4.0, 0.0001, 0.5);

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    swerveRequest.HeadingController = new PhoenixPIDController(4, 0, 0);

    swerveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();

    double xVelocity = xController.calculate(xError, 0.0);
    double yVelocity = yController.calculate(yError, 0.0);

    double maxVelocity = 4.0;
    double clampedXVelocity = Math.max(-maxVelocity, Math.min(maxVelocity, xVelocity));
    double clampedYVelocity = Math.max(-maxVelocity, Math.min(maxVelocity, yVelocity));
    swerveRequest = swerveRequest
    .withVelocityX(clampedXVelocity)
    .withVelocityY(clampedYVelocity)
    .withTargetDirection(targetPose.getRotation());

    drivetrain.setControl(swerveRequest);
  }

  @Override
  public void end(boolean interrupted) {
    swerveRequest = new FieldCentricFacingAngle()
      .withVelocityX(0.0)
      .withVelocityY(0.0);
  
      drivetrain.setControl(swerveRequest);
    }


  @Override
  public boolean isFinished() {
    Pose2d currentPose = drivetrain.getState().Pose;
      double xError = Math.abs(targetPose.getX() - currentPose.getX());
      double yError = Math.abs(targetPose.getY() - currentPose.getY());
      double headingError = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());

      return xError < POSITION_TOLERANCE && yError < POSITION_TOLERANCE && headingError < HEADING_TOLERANCE;
  }
}
