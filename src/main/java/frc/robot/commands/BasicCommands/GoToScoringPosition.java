// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Helpers.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GoToScoringPosition extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d targetPose;

  private final PIDController xController;
  private final PIDController yController;

  // Tolerance for position (meters) and heading (degrees)
  private static final double POSITION_TOLERANCE = 0.02;
  private static final double HEADING_TOLERANCE = Math.toRadians(1);
  
  private FieldCentricFacingAngle swerveRequest = new FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity);

  public GoToScoringPosition(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.targetPose = ScoreConditioningCalculator(true);
    this.xController = new PIDController(2, 0.0001, 0.0);
    this.yController = new PIDController(2, 0.0001, 0.0);
    
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    swerveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();

    double xVel = xController.calculate(xError, 0.0);
    double yVel = yController.calculate(yError, 0.0);

    double maxVelocity = 4.0;
    double clampedXVelocity = Math.max(-maxVelocity, Math.min(maxVelocity, xVel));
    double clampedYVelocity = Math.max(-maxVelocity, Math.min(maxVelocity, yVel));

    drivetrain.applyRequest(() -> swerveRequest
        .withVelocityX(clampedXVelocity)
        .withVelocityY(clampedYVelocity)
        .withTargetDirection(targetPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> new FieldCentricFacingAngle()
      .withVelocityX(0.0)
      .withVelocityY(0.0));
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drivetrain.getState().Pose;
    double xError = Math.abs(targetPose.getX() - currentPose.getX());
    double yError = Math.abs(targetPose.getY() - currentPose.getY());
    double headingError = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
    return xError < POSITION_TOLERANCE && yError < POSITION_TOLERANCE && headingError < HEADING_TOLERANCE;
  }

  Pose2d ScoreConditioningCalculator(boolean left)
  {
    double reefX = 4.48945; //meters
    double reefY = 4.0259; //meters
    double radius = 1.6256; //meters
    double angleoffset = 5;

    double targetX = 0;
    double targetY = 0;

    if(left)
    {
      angleoffset = angleoffset * -1;
    }

    double tagAngle = 0;
    // Get the tag ID for the visible April tag
    double tagId = LimelightHelpers.getFiducialID(Constants.Limelight.FRONT);
    if(tagId ==  18)
      tagAngle = 180;
    else if (tagId == 19)
      tagAngle = -120;
    else if(tagId ==  20)
      tagAngle = -60;
    else if (tagId == 21)
      tagAngle = 0;
    else if (tagId == 22)
      tagAngle = 60;
    else if(tagId ==  17)
      tagAngle = 120;
    else
      // Abandon ship!
      end(true);      

    double deltaangle = tagAngle + angleoffset;
    
    targetX = radius * Math.cos(Math.toRadians(deltaangle));
    targetY = radius * Math.sin(Math.toRadians(deltaangle));
    
    targetX = reefX + targetX;
    targetY = reefY + targetY; 
    
    Pose2d targetPos = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(tagAngle));
    return targetPos;
  }
}