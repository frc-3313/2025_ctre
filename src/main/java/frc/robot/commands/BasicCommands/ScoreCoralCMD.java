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
   if(coral.coralFullyAcquired() && !coral.coralPartiallyAcquired())
    {
      if (stateMachine.getScoreHeight() == 0)
      elevator.setHeight(Constants.Elevator.First);
    else if (stateMachine.getScoreHeight() == 1)
      elevator.setHeight(Constants.Elevator.Second);
    else if (stateMachine.getScoreHeight() == 2)
      elevator.setHeight(Constants.Elevator.Third);
    else if (stateMachine.getScoreHeight() == 3)
      elevator.setHeight(Constants.Elevator.Fourth); 
    }

  }

  @Override
  public void execute()
  {
    if(elevator.atSetpoint())
    coral.RunIntake(-.4);
  }

  @Override
  public void end(boolean interrupted) 
  {
    coral.StopIntake();
    //elevator.setHeight(Constants.Elevator.BottomPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    // if (timer.hasElapsed(1))
    // {
    //   return true;
    // }
    // if(!coral.coralFullyAcquired() && !timer.isRunning())
    // {
    //   timer.start();
    // }
    if(!coral.coralFullyAcquired() || coral.coralPartiallyAcquired())
      return true;
    return false;
  }
}
