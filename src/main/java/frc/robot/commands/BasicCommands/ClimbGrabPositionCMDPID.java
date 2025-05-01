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

public class ClimbGrabPositionCMDPID extends Command 
{
  public Timer timer;
  Climber climber;
  double position;
  StateMachine stateMachine;
  private final PIDController pidController;
  private double kp = .1;

  public ClimbGrabPositionCMDPID(Climber climber, StateMachine stateMachine) {
    this.climber = climber;
    this.stateMachine = stateMachine;
    this.pidController = new PIDController(kp, 0.0, 0.0);

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    if (DriverStation.getMatchTime() <= Constants.Climber.MaxMatchTime) 
    {
      timer = new Timer();
      timer.reset();
      timer.start();
      climber.Release();
      climber.setSpeed(.1, 0);
      climber.ReleaseWings();;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getMatchTime() <= Constants.Climber.MaxMatchTime) 
    {
      if (timer.hasElapsed(.75)) {
        climber.setSpeed(pidController.calculate(Constants.Climber.grabPos - climber.getEncoder(), 0), Constants.Climber.grabPos);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    stateMachine.setReadyToClimb(true);
    climber.stopWings();
    climber.setSpeed(0, Constants.Climber.climbPos);

    /*climber.atSetpoint();
    climber.setMotorBrake();*/
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.getMatchTime() > Constants.Climber.MaxMatchTime) 
    {
      return true;
    }
    if (timer.hasElapsed(.6)) 
    {
      return climber.atSetpoint();
    }
    return false;
    
  }
}
