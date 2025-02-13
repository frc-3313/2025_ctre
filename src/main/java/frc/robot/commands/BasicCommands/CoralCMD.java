// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.Constants;

public class CoralCMD extends Command 
{
  public Coral coral;

  
  public boolean timerStarted;

  public CoralCMD(Coral m_coral)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    coral = m_coral;
    addRequirements(coral); 
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    
    if(!coral.coralFullyAcquired())
    {
      coral.StopIntake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(!coral.coralPartiallyAcquired())
    {
      coral.RunIntake(.4);
    }
    if(coral.coralFullyAcquired())
    {
      coral.StopIntake();
     
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
      coral.StopIntake();
      
      SmartDashboard.putBoolean("intake is done", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(coral.coralFullyAcquired())
    {
     
      
        return true;
     
    }
    else
    {
      return false;
    }
    
  }
  @Override
  public InterruptionBehavior getInterruptionBehavior()
  {
    return Command.InterruptionBehavior.kCancelIncoming;
  }
}
