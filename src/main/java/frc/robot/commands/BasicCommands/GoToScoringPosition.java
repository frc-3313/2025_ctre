// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

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
  private final PIDController thetaController;
  
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

  private static final double POSITION_TOLERANCE = 0.05; // meters
  private static final double ROTATION_TOLERANCE = Math.toRadians(2); // radians
  private static final double MAX_SPEED = 4.0; // meters/sec
  private static final double MAX_ANGULAR_RATE = Math.toRadians(270); // 270°/s in rad/s (~4.71 rad/s)
  private final SwerveRequest.FieldCentricFacingAngle snapDrive = new FieldCentricFacingAngle()
  .withDeadband(Constants.OperatorConstants.LEFT_X_DEADBAND)
  .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

  public GoToScoringPosition(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.targetPose = ScoreConditioningCalculator(true);
    this.xController = new PIDController(2.50, 0.0001, 0.0);
    this.yController = new PIDController(2.50, 0.0001, 0.0);
    this.thetaController = new PIDController(2.0, 0.0001, 0.0);
    
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    thetaController.setTolerance(ROTATION_TOLERANCE);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    snapDrive.HeadingController = new PhoenixPIDController(10, 0, 0);

    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double thetaError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

    double xVel = xController.calculate(xError, 0.0);
    double yVel = yController.calculate(yError, 0.0);
    double angularVel = thetaController.calculate(thetaError, 0.0);

    xVel = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, xVel));
    yVel = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, yVel));
    angularVel = Math.max(-MAX_ANGULAR_RATE, Math.min(MAX_ANGULAR_RATE, angularVel));

    drivetrain.setControl(snapDrive.withTargetDirection(targetPose.getRotation())
    .withVelocityX(xVel) // Drive forward with negative Y (forward)
    .withVelocityY(yVel)); // Drive left with negative X (left)););

  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drivetrain.getState().Pose;
    double xError = Math.abs(targetPose.getX() - currentPose.getX());
    double yError = Math.abs(targetPose.getY() - currentPose.getY());
    double thetaError = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());

    return xError < POSITION_TOLERANCE && yError < POSITION_TOLERANCE && thetaError < ROTATION_TOLERANCE;
  }

  Pose2d ScoreConditioningCalculator(boolean left)
  {
    double targetX = 0;
    double targetY = 0;
    double angleoffset = Constants.Coral.angleoffset;

    if(left)
    {
      angleoffset = Constants.Coral.angleoffset * -1;
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
    
    targetX = Constants.Coral.scoreRadius * Math.cos(Math.toRadians(deltaangle));
    targetY = Constants.Coral.scoreRadius * Math.sin(Math.toRadians(deltaangle));
    
    targetX = Constants.Coral.blueReefX + targetX;
    targetY = Constants.Coral.blueReefY + targetY; 
    
    Pose2d targetPos = new Pose2d(targetX, targetY, new Rotation2d(deltaangle));
    return targetPos;
  }
}