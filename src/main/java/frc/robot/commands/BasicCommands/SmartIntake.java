// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.StateMachine;

public class SmartIntake extends Command {

  CommandSwerveDrivetrain drivetrain;
  StateMachine stateMachine;
  Coral coral;
  CommandXboxController controller;
  double desAngle;

  public SmartIntake(StateMachine stateMachine, Coral coral, CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
    this.stateMachine = stateMachine;
    this.coral = coral;
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(coral, drivetrain);
  }

  @Override
  public void initialize() 
  {
    coral.RunIntake(-.15);
  }

  @Override
  public void execute() 
  {
    Pose2d currentPos = drivetrain.getState().Pose;

    if(currentPos.getY() > 4.0259)
      desAngle = 36;
    else
      desAngle = -36;

    SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromDegrees(desAngle))
      .withVelocityX(controller.getLeftY() * Math.abs(controller.getLeftY()) * stateMachine.getMaxSpeed()) // Drive forward with negative Y (forward)
      .withVelocityY(controller.getLeftX()* Math.abs(controller.getLeftX()) * stateMachine.getMaxSpeed()); // Drive left with negative X (left)
    drivetrain.setControl(swerveRequest);
      
    SmartDashboard.putNumber("RobotX", currentPos.getX());
    SmartDashboard.putNumber("RobotY", currentPos.getY());
    SmartDashboard.putNumber("Rotation", currentPos.getRotation().getDegrees());
  }

  @Override
  public void end(boolean interrupted) 
  {
    coral.StopIntake();
  }

  @Override
  public boolean isFinished() 
  {
    if(coral.coralFullyAcquired() && !coral.coralPartiallyAcquired())
    {
      return true;
    }
    return false;
  }
}
