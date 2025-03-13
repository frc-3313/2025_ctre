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

  private static final double POSITION_TOLERANCE = 0.125; // meters
  private static final double ROTATION_TOLERANCE = Math.toRadians(2); // radians
  private static final double MAX_SPEED = 4.0; // meters/sec
  private static final double MAX_ANGULAR_RATE = Math.toRadians(270); // 270°/s in rad/s (~4.71 rad/s)

  //private final AprilTagFieldLayout fieldLayout;
  private final SwerveRequest.FieldCentricFacingAngle driveRequest = new FieldCentricFacingAngle()
  .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

  public GoToScoringPosition(CommandSwerveDrivetrain drivetrain, StateMachine stateMachine) {
    this.drivetrain = drivetrain;
    this.stateMachine = stateMachine;
    //this.fieldLayout = fieldLayout;
    this.xController = new PIDController(Constants.DrivebaseConstants.KPDrive, 0, 0);
    this.yController = new PIDController(Constants.DrivebaseConstants.KPDrive, 0, 0);

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    driveRequest.HeadingController = new PhoenixPIDController(Constants.DrivebaseConstants.KPDrive, 0, 0);
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
      //System.out.println("degrees :" + targetPose.getRotation());
      drivetrain.setControl(driveRequest
      .withVelocityX(-xVel)
      .withVelocityY(-yVel)
      .withTargetDirection(targetPose.getRotation()));

    }
  }

  @Override
  public void end(boolean interrupted) {
  drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
  drivetrain.setControl(driveRequest
    .withVelocityX(0)
    .withVelocityY(0)
    .withTargetDirection(targetPose.getRotation()));
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

    double reefX; //meters
    double reefY; //meters

    double radius = 1.434935988; //meters1.2
    double angleoffset = 8;//7.285188605;
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
    {
      reefX = 4.48667;
      reefY = 4.03;
    }
    else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
    {
      reefX = 13.05667;
      reefY = 4.03;
    }
    else
    {
      end(true);
      return null;
    }

    double targetX = 0;
    double targetY = 0;

    if(stateMachine.isScoreLeft())
    {
      angleoffset = angleoffset * -1;
    }
    double tagAngle = 0;
    // Get the tag ID for the visible April tag
    int tagId = (int)LimelightHelpers.getFiducialID(Constants.Limelight.FRONT);
    //Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagId);
    //fieldLayout.getTagPose(tagId);
    //System.out.println("tagPose: " + Units.radiansToDegrees(tagPose.get().getRotation().getAngle()));
    if(tagId ==  18 || tagId == 10)
      tagAngle = 0;
    else if (tagId == 19 || tagId == 9)
      tagAngle = -60;
    else if(tagId ==  20 || tagId == 8)
      tagAngle = -120;
    else if (tagId == 21 || tagId == 7)
      tagAngle = 180;
    else if (tagId == 22 || tagId == 6)
      tagAngle = 120;
    else if(tagId ==  17 || tagId == 11)
      tagAngle = 60;
    else
      // Abandon ship!
      return null;      
    
    double deltaangle = tagAngle + angleoffset + 180; //took out +180 for red side testing
    targetX = radius * Math.cos(Math.toRadians(deltaangle));
    targetY = radius * Math.sin(Math.toRadians(deltaangle));
    
    targetX = reefX + targetX;
    targetY = reefY + targetY; 
    System.out.println("gotoscore :" + targetX + ":" + targetY);
    return new Pose2d(targetX, targetY, Rotation2d.fromDegrees(tagAngle));
  
  }
}
