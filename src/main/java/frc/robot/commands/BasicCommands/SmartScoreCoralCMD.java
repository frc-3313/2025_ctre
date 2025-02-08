// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
import frc.robot.Constants;

public class SmartScoreCoralCMD extends Command 
{

  public Tilter tilter;
  public Shooter shooter;
  public Elevator elevator;

  public SmartScoreCoralCMD(Tilter tilter, Shooter shooter, Elevator elevator) 
  {
    this.tilter = tilter;
    this.shooter = shooter;
    this.elevator = elevator;
    addRequirements(tilter, shooter, elevator);
  }

  @Override
  public void initialize() 
  {
    shooter.SetShooterSpeed(2600);
    tilter.GoToPosition(Constants.Tilter.shootFromSpeaker);
  }

  @Override
  public void execute() 
  {
    elevator.GoToHeight();
  }

  @Override
  public void end(boolean interrupted) 
  {
  }

  @Override
  public boolean isFinished() {
    if(limelight.isTargetValid() && shooter.IsShooterAboveRPM() && tilter.atSetpoint())
    {
      shooter.MoveFeederDistance(300);
      if(shooter.FeederDone())
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
}
