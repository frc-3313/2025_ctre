// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;

import javax.lang.model.util.ElementScanner14;

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
    timer = new Timer();
    timer.reset();
  }

  @Override
  public void execute()
  {
    if((elevator.atSetpoint())){
      if (stateMachine.getScoreHeight() == 3)
        {
          coral.RunIntake(-20);
        }
      else {
        coral.RunIntake(-400);
      }
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    coral.StopIntake();
    elevator.setHeight(Constants.Elevator.BottomPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
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
