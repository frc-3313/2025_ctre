// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Coral;
import frc.robot.Constants;

public class SmartScoreCoralCMD extends Command 
{

  public Coral coral;
  public Elevator elevator;

  public SmartScoreCoralCMD(Coral coral, Elevator elevator) 
  {
    this.coral = coral;
    this.elevator = elevator;
    addRequirements(coral, elevator);
  }

  @Override
  public void initialize() 
  {
    coral.SetShooterSpeed(2600);
    tilter.GoToPosition(Constants.Tilter.shootFromSpeaker);//how should I use this?
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
    if(limelight.isTargetValid() && coral.IsShooterAboveRPM() && tilter.atSetpoint())
    {
      coral.MoveFeederDistance(300);
      if(coral.FeederDone())
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
