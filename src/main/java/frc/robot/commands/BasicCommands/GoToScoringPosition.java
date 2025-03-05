// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Helpers.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GoToScoringPosition extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d targetPose;

  private final PIDController xController;
  private final PIDController yController;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

  private static final double POSITION_TOLERANCE = 0.05; // meters
  private static final double ROTATION_TOLERANCE = Math.toRadians(2); // radians
  private static final double MAX_SPEED = 4.0; // meters/sec
  private static final double MAX_ANGULAR_RATE = Math.toRadians(270); // 270°/s in rad/s (~4.71 rad/s)
  //private final AprilTagFieldLayout fieldLayout;
  private final SwerveRequest.FieldCentricFacingAngle snapDrive = new FieldCentricFacingAngle()
  .withDeadband(Constants.OperatorConstants.LEFT_X_DEADBAND)
  .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

  public GoToScoringPosition(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    //this.fieldLayout = fieldLayout;
    this.targetPose = ScoreConditioningCalculator(true);
    this.xController = new PIDController(2.50, 0.0001, 0.0);
    this.yController = new PIDController(2.50, 0.0001, 0.0);

    
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    snapDrive.HeadingController = new PhoenixPIDController(2, 0, 0.1);
    snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
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

    xVel = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, xVel));
    yVel = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, yVel));

    drivetrain.setControl(snapDrive.withTargetDirection(Rotation2d.fromDegrees(0))
    .withVelocityX(drivetrain.getDriveY(xVel)) // Drive forward with negative Y (forward)
    .withVelocityY(drivetrain.getDriveX(yVel))); // Drive left with negative X (left)););
    System.out.println("target rotation" + targetPose.getRotation().getDegrees() + ":" + drivetrain.getState().Pose.getRotation().getDegrees());

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

    return xError < POSITION_TOLERANCE && yError < POSITION_TOLERANCE;
  }

  Pose2d ScoreConditioningCalculator(boolean left)
  {
    double reefX = 4.48945; //meters
    double reefY = 4.386; //meters
    double radius = 1.65046; //meters
    double angleoffset = 5.662;

    double targetX = 0;
    double targetY = 0;

    if(left)
    {
      angleoffset = angleoffset * -1;
    }

    double tagAngle = 0;
    // Get the tag ID for the visible April tag
    int tagId = (int)LimelightHelpers.getFiducialID(Constants.Limelight.FRONT);
    //Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagId);

    //fieldLayout.getTagPose(tagId);
    //System.out.println("tagPose: " + Units.radiansToDegrees(tagPose.get().getRotation().getAngle()));
    if(tagId ==  18)
      tagAngle = 0;
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
    
    Pose2d targetPos = new Pose2d(targetX, targetY, new Rotation2d(Math.toRadians(tagAngle)));
    return targetPos;
  }
}