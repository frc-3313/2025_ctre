// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.StateMachine;

public class CoralCMD extends Command 
{
  public Coral coral;
  public StateMachine stateMachine;
  
  double scoreSpeed;
  public boolean timerStarted;

  public CoralCMD(Coral m_coral,StateMachine stateMachine, double scoreSpeed)
  {
    coral = m_coral;
    this.stateMachine = stateMachine;
    this.scoreSpeed = scoreSpeed;
    addRequirements(coral); 
  }

  @Override
  public void initialize() 
  {
    coral.RunIntake(scoreSpeed * -1);
  }

  @Override
  public void execute() 
  {
    if(coral.coralFullyAcquired())
    {
      coral.RunIntake(-0.045);
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
      coral.StopIntake();
      SmartDashboard.putBoolean("intake is done", true);
  }

  @Override
  public boolean isFinished() {
    if(coral.coralFullyAcquired() && !coral.coralPartiallyAcquired())
    {
      return true;
    }
    else if (stateMachine.GetKillCommands()) 
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
    return Command.InterruptionBehavior.kCancelSelf;
  }
}
