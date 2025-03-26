// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
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
  private final PIDController rotController;

  private double offsetRightX = 0.054;
  private double offsetRightY = -0.001;
  private double offsetLeftX = 0.039; //0.057
  private double offsetLeftY = 0.0005; //0.016
  private double kp = .3;
  private double offsetX, offsetY;

  private double txError = 0.15, tyError = 1;

  public DriveToAprilTag(CommandSwerveDrivetrain swerveDrive, StateMachine stateMachine) {
    this.swerveDrive = swerveDrive;
    this.stateMachine = stateMachine;
    this.xController = new PIDController(kp, 0.0, 0.0);
    this.yController = new PIDController(kp, 0.0, 0.0);
    this.rotController = new PIDController(kp, 0.0, 0.0);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    stateMachine.SetReadyToScore(false);

    // Reset PID controllers
    xController.reset();
    yController.reset();
    rotController.reset();
    if(stateMachine.isScoreLeft())
    {
      limelight = Constants.Limelight.RIGHT;
      LimelightHelpers.setFiducial3DOffset(Constants.Limelight.RIGHT, offsetRightX, offsetRightY, 0);
      offsetX = offsetRightX;
      offsetY = offsetRightY;
    }
    else{
      limelight = Constants.Limelight.LEFT;
      LimelightHelpers.setFiducial3DOffset(Constants.Limelight.LEFT, offsetLeftX, offsetLeftY, 0);
      offsetX = offsetLeftX;
      offsetY = offsetLeftY;
    }
    // driveRequest.HeadingController = new PhoenixPIDController(4, 0, 0);
    // driveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    
  }

  @Override
  public void execute()
  {
    double tx = LimelightHelpers.getTX(limelight); // Horizontal offset to offset POI
    double ty = LimelightHelpers.getTY(limelight); // Vertical offset to offset POI
    double positions[] = LimelightHelpers.getBotPose_TargetSpace(limelight);
    double rot = positions[4];
    boolean tv = LimelightHelpers.getTV(limelight);

    tx += offsetX;
    ty += offsetY;

    if(tv)
    {
      double robotYVelocity = yController.calculate(tx, 0);
      double robotXVelocity = xController.calculate(ty, 0);
      double rotbotRot = rotController.calculate(rot, 0);

      driveRequest.VelocityY = robotYVelocity;
      driveRequest.VelocityX = -robotXVelocity;
      driveRequest.RotationalRate = -rotbotRot;

      swerveDrive.setControl(driveRequest);
    }

    SmartDashboard.putBoolean("drive to apirl rot", rotController.atSetpoint());
    SmartDashboard.putBoolean("drive to apirl ycont", rotController.atSetpoint());
    SmartDashboard.putBoolean("drive to apirl xcont", rotController.atSetpoint());

  }


  @Override
  public void end(boolean interrupted) {
    swerveDrive.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    swerveDrive.setControl(driveRequest
      .withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    if(LimelightHelpers.getTX(limelight) <= txError && 
    LimelightHelpers.getTY(limelight) <= tyError)
    {
      stateMachine.SetReadyToScore(true);
      return true;
    }
    return false;
  }
}
