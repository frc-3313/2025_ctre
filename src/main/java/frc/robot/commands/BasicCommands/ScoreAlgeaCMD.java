// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algea;
import frc.robot.subsystems.StateMachine;


public class ScoreAlgeaCMD extends Command 
{
  public Algea algea;
  public boolean timerStarted;
  public double speed;
  public boolean scoringAlgea;

  public ScoreAlgeaCMD(Algea m_algea, double speed)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    algea = m_algea;
    this.speed = speed;
    addRequirements(algea); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if (speed<0)
      scoringAlgea = true;
    else
      scoringAlgea = false;

    if(algea.algeaAcquired())
    {
      algea.StopIntake();
    }
    else 
    {
      algea.RunIntake(speed);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(algea.algeaAcquired() && !scoringAlgea)
    {
      algea.StopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    algea.StopIntake();
    SmartDashboard.putBoolean("intake is done", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(algea.algeaAcquired() && !scoringAlgea)
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
