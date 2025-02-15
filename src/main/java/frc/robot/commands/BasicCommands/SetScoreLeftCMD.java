// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;

public class SetScoreLeftCMD extends Command 
{

  public StateMachine stateMachine;
  public boolean left;
  
    public SetScoreLeftCMD(StateMachine stateMachine, boolean left)
    {
      // Use addRequirements() here to declare subsystem dependencies.
      this.stateMachine = stateMachine;
      this.left = left;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    stateMachine.setScoreLeft();
    left = true;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     
        return false;
    }
    
}
