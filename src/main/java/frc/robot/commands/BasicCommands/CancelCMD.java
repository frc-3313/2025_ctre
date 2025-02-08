// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Algea;
import edu.wpi.first.wpilibj.Timer;


public class CancelCMD extends InstantCommand {
  public Command CMD;

  /** Creates a new AmpScoreCMD. */
  public CancelCMD(Command CMD){
    this.CMD = CMD;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  // tilter.GoToPosition(Constants.Tilter.ampPosition); 
  // shooter.StartShooter();
    CMD.cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    
  // shooter.StopAllMotors(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

      return true;
  }
}
