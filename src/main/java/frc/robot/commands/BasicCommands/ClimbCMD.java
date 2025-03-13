// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.StateMachine;


public class ClimbCMD extends Command 
{

  public Timer timer;
  Climber climber;
  boolean climberHasStarted;
  private StateMachine stateMachine;
  public ClimbCMD(Climber climber, StateMachine stateMachine) {
    this.stateMachine = stateMachine;
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    timer = new Timer();
    timer.start();
    climberHasStarted = false;

    if (DriverStation.getMatchTime() <= Constants.Climber.MaxMatchTime) 
    {
      if(stateMachine.isReadyToClimb())
      {
        climber.Lock();
        climber.Raise();
        climberHasStarted = true;
      }
    }
    
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
    if (DriverStation.getMatchTime() > Constants.Climber.MaxMatchTime) 
    {
      return true;
    }
    if (!stateMachine.isReadyToClimb())
    {
      return true;
    }
    if (climberHasStarted)
      return climber.atSetpoint();
    else 
      return false;
  }
}
