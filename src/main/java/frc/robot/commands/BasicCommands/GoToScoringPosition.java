// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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

  public GoToScoringPosition(CommandSwerveDrivetrain drivetrain, double x, double y, double rotation) {
    this.drivetrain = drivetrain;
    this.targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
    
    this.xController = new PIDController(1.0, 0.0, 0.0);
    this.yController = new PIDController(1.0, 0.0, 0.0);
    this.thetaController = new PIDController(2.0, 0.0, 0.0);
    
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    thetaController.setTolerance(ROTATION_TOLERANCE);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
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

    drivetrain.applyRequest(() -> {
      double xVel = xController.calculate(xError, 0.0);
      double yVel = yController.calculate(yError, 0.0);
      double angularVel = thetaController.calculate(thetaError, 0.0);

      xVel = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, xVel));
      yVel = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, yVel));
      angularVel = Math.max(-MAX_ANGULAR_RATE, Math.min(MAX_ANGULAR_RATE, angularVel));

      return driveRequest
        .withVelocityX(xVel)
        .withVelocityY(yVel)
        .withRotationalRate(angularVel);
    });
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
}