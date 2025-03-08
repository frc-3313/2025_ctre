// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.StateMachine;


public class ClimbCMD extends Command 
{

  public Timer timer;
  Climber climber;
  double position;
  boolean climberHasStarted;
  private StateMachine stateMachine;
  public ClimbCMD(Climber climber, double m_position, StateMachine stateMachine) {
    this.stateMachine = stateMachine;
    this.climber = climber;
    this.position = m_position;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    timer = new Timer();
    timer.start();
    climberHasStarted = false;

    // elevator.setMotorAmp(80);
    if (DriverStation.getMatchTime() <= 15.0) 
    {
      climber.Lock();
      climber.Raise();
      climberHasStarted = true;
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
    //climber.atSetpoint();
    //climber.setMotorBrake();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if (DriverStation.getMatchTime() > 15.0) 
    {
      return true;
    }
    if (climberHasStarted)
      return climber.atSetpoint();
    else 
      return false;
  }
}
