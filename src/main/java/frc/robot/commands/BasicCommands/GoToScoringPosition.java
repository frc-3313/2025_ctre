// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StateMachine;
import frc.robot.Helpers.*;

public class GoToScoringPosition extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final StateMachine stateMachine;
  private Pose2d targetPose;

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
  .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

  public GoToScoringPosition(CommandSwerveDrivetrain drivetrain, StateMachine stateMachine) {
    this.drivetrain = drivetrain;
    this.stateMachine = stateMachine;
    //this.fieldLayout = fieldLayout;
    this.xController = new PIDController(4.0, 0.0001, 0.5);
    this.yController = new PIDController(4.0, 0.0001, 0.5);

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    snapDrive.HeadingController = new PhoenixPIDController(4, 0, 0);
    this.targetPose = ScoreConditioningCalculator(false);

    snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
    
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    if(targetPose != null)
    {
      double xError = targetPose.getX() - currentPose.getX();
      double yError = targetPose.getY() - currentPose.getY();

      double xVel = xController.calculate(xError, 0.0);
      double yVel = yController.calculate(yError, 0.0);

      xVel = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, xVel));
      yVel = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, yVel));
      System.out.println("degrees :" + targetPose.getRotation());
      drivetrain.setControl(snapDrive
      .withVelocityX(xVel)
      .withVelocityY(yVel)
      .withTargetDirection(targetPose.getRotation()));
    }
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
    if(targetPose == null)
      return true;
    return xError < POSITION_TOLERANCE && yError < POSITION_TOLERANCE;
  }

  Pose2d ScoreConditioningCalculator(boolean left)
  {
    double reefX; //meters
    double reefY; //meters

    double radius = 1.60046; //meters1.2
    double angleoffset = 5.66;
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
    {
      reefX = 4.84505;
      reefY = 4.02;
    }
    else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
    {
      reefX = 4.84505;
      reefY = 4.0259;
    }
    else
    {
      end(true);
      return null;
    }

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
      tagAngle = 180;
    else if (tagId == 19)
      tagAngle = 120;
    else if(tagId ==  20)
      tagAngle = 60;
    else if (tagId == 21)
      tagAngle = 0;
    else if (tagId == 22)
      tagAngle = -60;
    else if(tagId ==  17)
      tagAngle = -120;
    else
      // Abandon ship!
      return null;      
    
    double deltaangle = tagAngle + angleoffset;
    targetX = radius * Math.cos(Math.toRadians(deltaangle));
    targetY = radius * Math.sin(Math.toRadians(deltaangle));
    
    targetX = reefX + targetX;
    targetY = reefY + targetY; 
    return new Pose2d(targetX, targetY, Rotation2d.fromDegrees(tagAngle));
  
  }
}
