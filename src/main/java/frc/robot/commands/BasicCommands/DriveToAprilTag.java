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
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class DriveToAprilTag extends Command {

  private final CommandSwerveDrivetrain swerveDrive;
  private final StateMachine stateMachine;
  private String limelight;

  // private final SwerveRequest.FieldCentricFacingAngle driveRequest = new FieldCentricFacingAngle()
  // .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

  private final PIDController xController;
  private final PIDController yController;

  private double offsetX = 0.012;
  private double offsetY = -0.055;
  private double rot = 0;

  private final double kP = .1;

  private double txError = 0.15, tyError = 1;

  public DriveToAprilTag(CommandSwerveDrivetrain swerveDrive, StateMachine stateMachine) {
    this.swerveDrive = swerveDrive;
    this.stateMachine = stateMachine;

    this.xController = new PIDController(kP, 0.0, 0.0);
    this.yController = new PIDController(kP, 0.0, 0.0);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    // Reset PID controllers
    xController.reset();
    yController.reset();
    if(stateMachine.isScoreLeft())
      limelight = Constants.Limelight.FRONT;
    else{
      limelight = Constants.Limelight.RIGHT;
        offsetX *= -1;
      }

    // driveRequest.HeadingController = new PhoenixPIDController(4, 0, 0);
    // driveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    LimelightHelpers.setFiducial3DOffset(Constants.Limelight.RIGHT, offsetX, offsetY, 0);
  }

  @Override
  public void execute()
  {
    double tx = LimelightHelpers.getTX(limelight); // Horizontal offset to offset POI
    double ty = LimelightHelpers.getTY(limelight); // Vertical offset to offset POI
    boolean tv = LimelightHelpers.getTV(limelight);

    tx += offsetX;
    ty += offsetY;

    if(tv)
    {
      double robotYVelocity = yController.calculate(tx, 0);
      double robotXVelocity = xController.calculate(ty, 0);

      Rotation2d currentHeading = swerveDrive.getState().Pose.getRotation();

      driveRequest.VelocityY = robotYVelocity;
      driveRequest.VelocityX = -robotXVelocity;
      // driveRequest.TargetDirection = Rotation2d.fromDegrees(rot);

      swerveDrive.setControl(driveRequest);
    }
    else
    {
      return;
    }
  }


  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return LimelightHelpers.getTX(Constants.Limelight.RIGHT) <= txError && 
      LimelightHelpers.getTY(Constants.Limelight.RIGHT) <= tyError;
  }
}
