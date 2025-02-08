// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


public class ElevatorGoPosition extends Command {
  public Elevator elevator;
  public double position;
  /** Creates a new AmpScoreCMD. */
  public ElevatorGoPosition(Elevator m_Elevator, double m_position){
    
    elevator = m_Elevator;
    position = m_position;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
      elevator.GoToHeight(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(elevator.atSetpoint())
      return true;
    else  
      return false;
  }
}
