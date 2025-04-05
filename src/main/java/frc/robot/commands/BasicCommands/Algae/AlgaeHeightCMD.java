// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class AlgaeHeightCMD extends Command 
{
  
  public Timer timer;
  public Elevator elevator;
  public Boolean endBoolean;
  public StateMachine stateMachine;
  public boolean setHeight;

  public AlgaeHeightCMD(Elevator elevator, StateMachine stateMachine, boolean height) {
    this.elevator = elevator;
    this.stateMachine = stateMachine;
    this.setHeight = height;
    addRequirements(elevator);
  }

  @Override
  public void initialize() 
  {
    if(setHeight)
    {
      if (stateMachine.getAlgaeHeight() == 0)
        elevator.setHeight(Constants.Elevator.AlgaeFirst);
      else if (stateMachine.getAlgaeHeight() == 1)
        elevator.setHeight(Constants.Elevator.AlgaeSecond);
    }
    else
    {
      elevator.setHeight(Constants.Elevator.AlgaeScore);
    }
  }

  @Override
  public void execute()
  {
  
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
