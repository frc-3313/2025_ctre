// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorEject extends Command 
{
  
  public Elevator elevator;
  public StateMachine stateMachine;

  public ElevatorEject(Elevator elevator, StateMachine stateMachine) {
    this.elevator = elevator;
    this.stateMachine = stateMachine;
    addRequirements(elevator);
  }

  @Override
  public void initialize() 
  {
    elevator.setHeight(Constants.Elevator.Second);
    
  }

  @Override
  public void execute()
  {
  
  }

  @Override
  public void end(boolean interrupted) 
  {
    elevator.setHeight(Constants.Elevator.BottomPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
