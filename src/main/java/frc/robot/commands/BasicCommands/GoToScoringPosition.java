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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StateMachine;
import frc.robot.LimelightHelpers;

public class GoToScoringPosition extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final StateMachine stateMachine;
  private Pose2d targetPose;
  private boolean scoreLeft;
  private int tagID;

  double blueReefX = 4.48945, blueReefY = 4.0259;//DO NOT CHANGE EVER
  double redReefX = 13.0683, redReefY = 4.0259; //DO NOT CHANGE EVER

  double blueXOffSet = 0, blueYOffset = 0;
  double redXOffSet = 0, redYOffset = 0;
  double reefRadius = 1.06045;

  //adjustments tips
  //if the distance to the reef is off adjust the radius
  //if the robot of shift all the way around adjust offset for reef

  private final PIDController xController;
  private final PIDController yController;

  private static final double POSITION_TOLERANCE = 0.01; // meters
  private static final double ROTATION_TOLERANCE = Math.toRadians(1); // radians
  private static final double MAX_SPEED = 4.0; // meters/sec
  private static final double MAX_ANGULAR_RATE = Math.toRadians(270); // 270°/s in rad/s (~4.71 rad/s)

  //private final AprilTagFieldLayout fieldLayout;
  private final SwerveRequest.FieldCentricFacingAngle driveRequest = new FieldCentricFacingAngle()
  .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

  public GoToScoringPosition(CommandSwerveDrivetrain drivetrain, StateMachine stateMachine) {
    this.drivetrain = drivetrain;
    this.stateMachine = stateMachine;
    //this.fieldLayout = fieldLayout;
    this.xController = new PIDController(4.0, 0, 0);
    this.yController = new PIDController(4.0, 0, 0);

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    driveRequest.HeadingController = new PhoenixPIDController(4, 0, 0);
    driveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    xController.reset();
    yController.reset();
    if(stateMachine.isScoreLeft()){scoreLeft = true;}
    else{scoreLeft = false;}

    if(scoreLeft)
    {
      LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Constants.Limelight.RIGHT);
      if(fiducials.length > 0)
      {
        double tagIdDouble = fiducials[0].id;
        tagID = (int) tagIdDouble;
      }
      else
      {
        return;
      }
    }
    else
    {
      LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Constants.Limelight.FRONT);
      if(fiducials.length > 0)
      {
        double tagIdDouble = fiducials[0].id;
        tagID = (int) tagIdDouble;
      }
      else
      {
        return;
      }
    }
    
    targetPose = RobotPositionCalculator(tagID);
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

      drivetrain.setControl(driveRequest
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
    if(targetPose == null)
      return true;
    Pose2d currentPose = drivetrain.getState().Pose;
    double xError = Math.abs(targetPose.getX() - currentPose.getX());
    double yError = Math.abs(targetPose.getY() - currentPose.getY());
    
    return xError < POSITION_TOLERANCE && yError < POSITION_TOLERANCE;
  }

  Pose2d RobotPositionCalculator(int tagID)
  {
    Pose2d tagInfo = GetTagInfo(tagID);

    double littleTheta = Math.atan(0.1643/reefRadius);
    littleTheta = littleTheta + tagInfo.getRotation().getDegrees();
    double hype = Math.sqrt((reefRadius * reefRadius) + (0.1642 * 0.1643));
    double theta = tagInfo.getRotation().getDegrees() - littleTheta;
    
    double x = hype * Math.cos(Math.toRadians(theta));
    double y = hype * Math.sin(Math.toRadians(theta));
    
    Pose2d robotTargetPos = new Pose2d(blueReefX + x, blueReefY + y, Rotation2d.fromDegrees(tagInfo.getRotation().getDegrees() - 180));
    return robotTargetPos;
  }

  Pose2d GetTagInfo(int tagID)
  {
    double xPos = 0, yPos = 0, rot = 0;

    if(tagID == 6){xPos = 530.49; yPos = 130.17; rot = 300;}
    else if(tagID == 7){xPos = 546.87 ; yPos = 158.50; rot = 0;}
    else if(tagID == 8){xPos = 530.49 ; yPos = 186.83; rot = 60;}
    else if(tagID == 9){xPos = 497.77; yPos = 186.83; rot = 120;}
    else if(tagID == 10){xPos = 481.39; yPos = 158.50; rot = 180;}
    else if(tagID == 11){xPos = 497.77; yPos = 130.17; rot = 240;}
    else if(tagID == 17){xPos =160.39; yPos = 130.17; rot = 240;}
    else if(tagID == 18){xPos = 144.00; yPos = 158.50; rot = 180;}
    else if(tagID == 19){xPos = 160.39; yPos = 186.83; rot = 120;}
    else if(tagID == 20){xPos = 193.10 ; yPos = 186.83; rot = 60;}
    else if(tagID == 21){xPos = 209.49; yPos = 158.50; rot = 0;}
    else if(tagID == 22){xPos = 193.10; yPos = 130.17; rot = 300;}
    else{xPos = drivetrain.getState().Pose.getX(); yPos = drivetrain.getState().Pose.getY(); rot = drivetrain.getState().Pose.getRotation().getDegrees();}

    return new Pose2d(xPos, yPos, Rotation2d.fromDegrees(rot));
  }
}
