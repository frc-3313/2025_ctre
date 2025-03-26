// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.StateMachine;


public class ClimbCMDPID extends Command 
{

  public Timer timer;
  Climber climber;
  private final PIDController pidController;
  private double kp = .1;
  private double kd = .00;

  boolean climberHasStarted;
  private StateMachine stateMachine;
  public ClimbCMDPID(Climber climber, StateMachine stateMachine) {
    this.stateMachine = stateMachine;
    this.climber = climber;
    this.pidController = new PIDController(kp, 0.0, kd);

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
      // if(stateMachine.isReadyToClimb())
      // {
        climber.Lock();
        climber.setSpeed(pidController.calculate(Constants.Climber.climbPos - climber.getEncoder(), 0), Constants.Climber.climbPos);
        climberHasStarted = true;
      // }
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (DriverStation.getMatchTime() <= Constants.Climber.MaxMatchTime) 
    {
      // if(stateMachine.isReadyToClimb())
      // {
        climber.setSpeed(pidController.calculate(Constants.Climber.climbPos - climber.getEncoder(), 0), Constants.Climber.climbPos);
      // }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    climber.setSpeed(0.0, Constants.Climber.climbPos);

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
