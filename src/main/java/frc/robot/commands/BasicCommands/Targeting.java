// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Helpers.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Targeting extends Command 
{

  public Timer timer;
  double position;
  boolean alingended;
  CommandSwerveDrivetrain drivetrain;
  private double targetAngle;

  public Targeting(CommandSwerveDrivetrain commandSwerveDrivetrain, double DoubleRobotAngle)
  {
    drivetrain = commandSwerveDrivetrain;
    targetAngle = DoubleRobotAngle;
    addRequirements(commandSwerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
   //left=270
   //down=0
   //right=90
   //up=180
    SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withVelocityX(0.0) // No translational movement
      .withVelocityY(0.0)
      .withTargetDirection(Rotation2d.fromDegrees(targetAngle)); // Rotate to target
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {   
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
