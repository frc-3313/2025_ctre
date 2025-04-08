// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;

public class CoralScoreDrive extends Command {

  StateMachine stateMachine;
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController controller;
  private final SwerveRequest.FieldCentricFacingAngle snapDrive = new FieldCentricFacingAngle()
  .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

  double desAngle;
  double targetReefCenterX;
  double targetReefCenterY;
  boolean isRedAlliance;

  //Roation Calculation Variables
  static final double blueReefCenterX = 176.75 * 0.0254; //4.49 meters
  static final double blueReefCenterY = 158.5 * 0.0254; //4.03 meters
  static final double redReefCenterX = 13.05;
  static final double redReefCenterY = 4.03;

  public CoralScoreDrive(StateMachine stateMachine, CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
    this.stateMachine = stateMachine;
    this.drivetrain = drivetrain;
    this.controller = controller; 
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {

    snapDrive.HeadingController = new PhoenixPIDController(Constants.DrivebaseConstants.KPSteer, 0, 0);
    snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    isRedAlliance = DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);
    if (isRedAlliance) {
        targetReefCenterX = redReefCenterX;
        targetReefCenterY = redReefCenterY;
    } else {
        targetReefCenterX = blueReefCenterX;
        targetReefCenterY = blueReefCenterY;
    }
  }

  @Override
  public void execute() 
  {
    Pose2d curPose = drivetrain.getState().Pose;
    desAngle = GetRotationAngle(curPose.getX(), curPose.getY());

    drivetrain.setControl(snapDrive.withTargetDirection(Rotation2d.fromDegrees(desAngle))
    .withVelocityX(-controller.getLeftY() * Math.abs(controller.getLeftY()) * stateMachine.getMaxSpeed()) // Drive forward with negative Y (forward)
    .withVelocityY(-controller.getLeftX()* Math.abs(controller.getLeftX()) * stateMachine.getMaxSpeed())); // Drive left with negative X (left)););
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public boolean isFinished() {
    return false;
  }


  private double GetRotationAngle(double x, double y)
  {
    double deltaX = x - targetReefCenterX;
    double deltaY = y - targetReefCenterY;

    double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
    if(isRedAlliance)
    {
      if(angle > 150 || angle < -150)
        return 180;
      else if(angle > -150 && angle <= -90)
        return -120;
      else if(angle > -90 && angle <= -30)
        return -60;
      else if(angle > -30 && angle <= 30)
        return 0.01;
      else if(angle > 30 && angle <= 90)
        return 60;
      else
        return 120;
      }
      else
    {
      if(angle > 150 || angle < -150)
        return 0;
      else if(angle > -150 && angle <= -90)
        return 60;
      else if(angle > -90 && angle <= -30)
        return 120;
      else if(angle > -30 && angle <= 30)
        return 180;
      else if(angle > 30 && angle <= 90)
        return -120;
      else
        return -60;
    }
  }

}