// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class AlgaeScore extends Command 
{
  
  public Timer timer;
  public Algae algae;
  public Boolean endBoolean;
  public StateMachine stateMachine;
  public Elevator elevator;
  private final PIDController pidController;
  private double kp = .1;
  private double kd = .00;

  public AlgaeScore(Algae algae, Elevator elevator, StateMachine stateMachine) {
    timer = new Timer();
    timer.reset();
    timer.start();
    this.algae = algae;
    this.stateMachine = stateMachine;
    this.elevator = elevator;
    this.pidController = new PIDController(kp, 0.0, kd);

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

    stateMachine.SetReadyToScore(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {

    return false;
  }
}
