
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMachine extends SubsystemBase 
{
  private int scoreHeight;
  private boolean scoreLeft;
  private boolean scoreRight;
  private boolean killCommands;
  //Initialization

  public StateMachine() 
  {    
    scoreHeight = 0;
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
    scoreRight = false;
  } 

  public void setScoreRight()
  {
    scoreRight = true;
    scoreLeft = false;
  }
  public boolean getScoreRight()
  {
    return scoreRight;
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
}
