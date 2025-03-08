// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class DriveToAprilTag extends Command {

  private final CommandSwerveDrivetrain swerveDrive;

  private final SwerveRequest.FieldCentricFacingAngle driveRequest = new FieldCentricFacingAngle()
  .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

  private final PIDController xController;
  private final PIDController yController;

  private double offsetX = 0;
  private double offsetY = 0;
  private double offsetZ = 0;
  private double rot = 0;

  private final double kP = 2;

  public DriveToAprilTag(CommandSwerveDrivetrain swerveDrive) {
    this.swerveDrive = swerveDrive;

    this.xController = new PIDController(kP, 0.0, 0.0);
    this.yController = new PIDController(kP, 0.0, 0.0);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    // Reset PID controllers
    xController.reset();
    yController.reset();

    driveRequest.HeadingController = new PhoenixPIDController(4, 0, 0);
    driveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    LimelightHelpers.setFiducial3DOffset(Constants.Limelight.RIGHT, offsetX, offsetY, offsetZ);
  }

  @Override
  public void execute()
  {
    double tx = LimelightHelpers.getTX(Constants.Limelight.RIGHT); // Horizontal offset to offset POI
    double ty = LimelightHelpers.getTY(Constants.Limelight.RIGHT); // Vertical offset to offset POI
    boolean tv = LimelightHelpers.getTV(Constants.Limelight.RIGHT);

    if(tv)
    {
      double robotYVelocity = ty * kP;
      double robotXVelocity = tx * kP;

      Rotation2d currentHeading = swerveDrive.getState().Pose.getRotation();
      double fieldXVelocity = robotXVelocity * currentHeading.getCos() - robotYVelocity * currentHeading.getSin();
      double fieldYVelocity = robotXVelocity * currentHeading.getSin() + robotYVelocity * currentHeading.getCos();

      driveRequest.VelocityX = fieldXVelocity;
      driveRequest.VelocityY = fieldYVelocity;
      driveRequest.TargetDirection = Rotation2d.fromDegrees(rot);

      swerveDrive.setControl(driveRequest);
    }
    else
    {
      return;
    }
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
