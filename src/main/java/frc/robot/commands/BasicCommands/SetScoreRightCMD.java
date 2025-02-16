// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;

public class SetScoreRightCMD extends Command 
{

  public StateMachine stateMachine;
  public boolean right;
  
    public SetScoreRightCMD(StateMachine stateMachine, boolean right)
    {
      // Use addRequirements() here to declare subsystem dependencies.
      this.stateMachine = stateMachine;
      this.right = right;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    stateMachine.setScoreRight();
    right = true;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     
        return false;
    }
    
}
