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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;

public class SmartIntake extends Command {

  CommandSwerveDrivetrain drivetrain;
  StateMachine stateMachine;
  Coral coral;
  CommandXboxController controller;
  double desAngle;
  private final SwerveRequest.FieldCentricFacingAngle snapDrive = new FieldCentricFacingAngle()
  .withDeadband(Constants.OperatorConstants.LEFT_X_DEADBAND)
  .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);
  

  public SmartIntake(StateMachine stateMachine, Coral coral, CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
    this.stateMachine = stateMachine;
    this.coral = coral;
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(coral, drivetrain);
    snapDrive.HeadingController = new PhoenixPIDController(20, 0, 0);
    snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

  }

  @Override
  public void initialize() 
  {
    coral.RunIntake(-10);
  }

  @Override
  public void execute() 
  {
        coral.RunIntake(-10);

    Pose2d currentPos = drivetrain.getState().Pose;
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
    {
      if(currentPos.getY() > 4.0259)
        desAngle = 126 - 90;
      else
        desAngle = -126 + 90;
    }
    else
    {    
      if(currentPos.getY() > 4.0259)
        desAngle = -54;
      else
        desAngle = 54;
    }

  
    
    drivetrain.setControl(snapDrive.withTargetDirection(Rotation2d.fromDegrees(desAngle))
      .withVelocityX(drivetrain.getDriveY(-controller.getLeftY()) * stateMachine.getMaxSpeed()) // Drive forward with negative Y (forward)
      .withVelocityY(drivetrain.getDriveX(-controller.getLeftX()) * stateMachine.getMaxSpeed())); // Drive left with negative X (left));

    SmartDashboard.putNumber("RobotX", currentPos.getX());
    SmartDashboard.putNumber("RobotY", currentPos.getY());
    SmartDashboard.putNumber("Rotation", currentPos.getRotation().getDegrees());
    SmartDashboard.putNumber("DesiredAngle", desAngle);
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
