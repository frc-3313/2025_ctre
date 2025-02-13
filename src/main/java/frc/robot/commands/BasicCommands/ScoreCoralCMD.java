// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;

public class ScoreCoralCMD extends Command {
  
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
    endBoolean = false;
    if (stateMachine.getScoreHeight() == 0)
      elevator.GoToHeight(Constants.Elevator.elevatorFirst);
    else if (stateMachine.getScoreHeight() == 1)
      elevator.GoToHeight(Constants.Elevator.elevatorSecond);
    else if (stateMachine.getScoreHeight() == 2)
      elevator.GoToHeight(Constants.Elevator.elevatorThird);
    else if (stateMachine.getScoreHeight() == 3)
      elevator.GoToHeight(Constants.Elevator.elevatorFourth);
  }

  @Override
  public void execute()
  {
    if (elevator.atSetpoint())
      coral.RunIntake(-5);

  }

  @Override
  public void end(boolean interrupted) 
  {
    coral.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(coral.IsShooterAboveRPM() && tilter.atSetpoint())
    {
      if (!endBoolean)
      {
        coral.MoveFeederDistance(10);
        endBoolean = true;
      }
      return false;
    }
    else if(coral.FeederDone() && endBoolean)
    {
      return true;
    }
    else{
      return false;
    }
  }
}
