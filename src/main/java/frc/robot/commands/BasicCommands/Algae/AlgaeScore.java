// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class AlgaeScore extends Command 
{
  
  public Algae algae;
  public Boolean endBoolean;
  public StateMachine stateMachine;
  public Elevator elevator;

  public AlgaeScore(Algae algae, Elevator elevator, StateMachine stateMachine) {

    this.algae = algae;
    this.stateMachine = stateMachine;
    this.elevator = elevator;

    addRequirements(algae, elevator);
  }

  @Override
  public void initialize() 
  {
    algae.setPos(Constants.Algae.TilterScorePos);

  }

  @Override
  public void execute()
  {
    if(algae.tilterAtSetpoint())
    {
      algae.RunIntake(-10);
    }
  }

  @Override
  public void end(boolean interrupted) 
  {

    elevator.setHeight(Constants.Elevator.BottomPosition);
    algae.setPos(Constants.Algae.TilterStorePos);
    algae.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {

    return false;
  }
}
