
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

public class StateMachine extends SubsystemBase 
{
  private int scoreHeight;
  private boolean scoreLeft;
  private boolean killCommands;
  private boolean RunIntake;
  private boolean ElevRaised;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private boolean Manual;
  //Initialization

  public StateMachine() 
  {    
    scoreHeight = 0;
    Manual = false;
  }

  public void setScoreHeight(int input)
  {
    scoreHeight = input;
  }

  public int getScoreHeight()
  {
    return scoreHeight;
  }

  public void setScoreLeft()
  {
    scoreLeft = true;
  } 

  public void setScoreRight()
  {
    scoreLeft = false;
  }
  
  public boolean getScoreLeft()
  {
    return scoreLeft;
  }
  public void SetKillCommands(boolean killcommands)
  {
    killCommands = killcommands;
  }
  public boolean GetKillCommands()
  {
    return killCommands;
  }
  public void setIntake(boolean RunIntake)
  {
    this.RunIntake = RunIntake;
  }
  public Trigger runIntake()
  {
    return new Trigger(() -> RunIntake);
  }
  public Command IntakeCMD(boolean intakeBool)
  {
    return this.runOnce(() -> this.setIntake(intakeBool));
    
  }
  public double getMaxSpeed()
  {
    if(getElevRaised())
    {
      return MaxSpeed/8;
    }
    else
    {
      return MaxSpeed;
    }
  }
  public void setElevRaised(boolean input)
  {
    ElevRaised = input;
  }
  public boolean getElevRaised()
  {
    return ElevRaised;
  }
  public void setManual(boolean input)
  {
    Manual = input;
  }
  public boolean getManual()
  {
    return Manual;
  }
  public Trigger IsManual()
  {
    return new Trigger(() -> Manual);
  }
}
