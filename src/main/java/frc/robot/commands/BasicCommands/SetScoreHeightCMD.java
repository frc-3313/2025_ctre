// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;

public class SetScoreHeightCMD extends Command 
{

  public StateMachine stateMachine;
  public int height;

  public SetScoreHeightCMD(StateMachine stateMachine, int height)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stateMachine = stateMachine;
    this.height = height;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    stateMachine.setScoreHeight(height);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     
        return true;
    }
    
}
