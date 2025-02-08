// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class ZeroGyro extends Command {
 
  public SwerveSubsystem swerveSubsystem;
  /** Creates a new AmpScoreCMD. */
  public ZeroGyro(SwerveSubsystem m_SwerveSubsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSubsystem = m_SwerveSubsystem;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  // tilter.GoToPosition(Constants.Tilter.ampPosition); 
  // shooter.StartShooter();
   swerveSubsystem.zeroGyro();
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

    
  // shooter.StopAllMotors(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
      return true;
  }
}
