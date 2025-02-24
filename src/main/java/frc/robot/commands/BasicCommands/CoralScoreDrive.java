// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StateMachine;

public class CoralScoreDrive extends Command {

  StateMachine stateMachine;
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController controller;

  double desRot;

  //Roation Calculation Variables
  static final double blueReefCenterX = 176.75 * 0.0254;
  static final double blueReefCenterY = 158.5 * 0.0254;

  public CoralScoreDrive(StateMachine stateMachine, CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
    this.stateMachine = stateMachine;
    this.drivetrain = drivetrain;
    this.controller = controller; 
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    Pose2d curPose = drivetrain.getState().Pose;
    desRot = GetRotationAngle(curPose.getX(), curPose.getY());

    SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromDegrees(desRot))
      .withVelocityX(controller.getLeftY() * Math.abs(controller.getLeftY()) * stateMachine.getMaxSpeed())
      .withVelocityY(controller.getLeftX()* Math.abs(controller.getLeftX()) * stateMachine.getMaxSpeed());

    drivetrain.setControl(swerveRequest);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }


  double GetRotationAngle(double x, double y)
  {
    double deltaX = x - blueReefCenterX;
    double deltaY = y - blueReefCenterY;

    double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));

    if(angle < 0)
      angle += 360;

    if(angle <= 30 || angle > 330)
      return 0;
    else if(angle > 30 && angle <= 90)
      return 60;
    else if(angle > 90 && angle <= 150)
      return 120;
    else if(angle > 150 && angle <= 210)
      return 180;
    else if(angle > 210 && angle <= 270)
      return 240;
    else
      return 300;
  }
}
