// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands.Algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;

public class AlgaeIntake extends Command 
{
  public Algae algae;
  public Elevator elevator;
  public StateMachine stateMachine;
  

  public AlgaeIntake(Algae m_algae,Elevator elevator, StateMachine stateMachine)
  {
    this.algae = m_algae;
    this.elevator = elevator;
    this.stateMachine = stateMachine;

    addRequirements(algae, elevator); 
  }

  @Override
  public void initialize() 
  {
    algae.setPos(Constants.Algae.TilterIntakePos);
    algae.RunIntake(10);
  }

  @Override
  public void execute() 
  {

  }

  @Override
  public void end(boolean interrupted) 
  {
      algae.StopIntake();
      algae.setPos(Constants.Algae.TilterStorePos);
      elevator.setHeight(Constants.Elevator.BottomPosition);
  }

  @Override
  public boolean isFinished() {
    if(algae.AlgaeAcquired())
    {
      return true;
    }
    else
    {
      return false;
    }
    
  }
}
