// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class ScoreCoralCMD extends Command 
{
  
  public Timer timer;
  public Coral coral;
  public Elevator elevator;
  public Boolean endBoolean;
  public StateMachine stateMachine;

  public ScoreCoralCMD(Coral coral, Elevator elevator, StateMachine stateMachine) {
    this.coral = coral;
    this.elevator = elevator;
    this.stateMachine = stateMachine;
    addRequirements(coral, elevator);
  }

  @Override
  public void initialize() 
  {
    if(stateMachine.IsReadyToScore())
    {
      timer = new Timer();
      timer.reset();
    }
  }

  @Override
  public void execute()
  {
    if(stateMachine.IsReadyToScore())
    {
      if((elevator.atSetpoint()))
      {
        if (stateMachine.getScoreHeight() == 3)
        {
          coral.RunIntake(-25);
        }
        else if (stateMachine.getScoreHeight() == 0)
        {
          coral.RunIntake(-15);
        }
        else 
        {
          coral.RunIntake(-30);
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    if(stateMachine.IsReadyToScore())
    {
      coral.StopIntake();
    }
    elevator.setHeight(Constants.Elevator.BottomPosition);
    stateMachine.SetReadyToScore(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(!stateMachine.IsReadyToScore())
    {
      return true;
    }
    if (stateMachine.getScoreHeight() == 3 && !coral.coralFullyAcquired())
    {
      if (timer.isRunning() && timer.hasElapsed(.2))
      {
        return true;
      }
      else
      {
        timer.start();
      }
    }
    else if(!coral.coralFullyAcquired())
    {
      return true;
    }
    return false;
  }
}
