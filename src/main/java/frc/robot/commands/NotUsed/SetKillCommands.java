// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NotUsed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.StateMachine;

public class SetKillCommands extends Command 
{

  StateMachine stateMachine;
  boolean setKillBoolean;

  public SetKillCommands(StateMachine stateMachine, Boolean setKillBoolean) {
    this.setKillBoolean = setKillBoolean;
    this.stateMachine = stateMachine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    stateMachine.SetKillCommands(setKillBoolean);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

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
    return true;
  }
}
