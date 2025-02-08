// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilter;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class JognoteCMD extends Command 
{
  public Intake intake;
  public Shooter shooter;
  public Tilter tilter;
  public Timer timer;
  public boolean timerStarted;

  public JognoteCMD(Shooter m_Shooter)
  {
    // Use addRequirements() here to declare subsystem dependencies.
  
    shooter = m_Shooter;
    
    addRequirements(shooter); 
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    
    if(shooter.noteToClose())
    {
      shooter.MoveFeederDistance(Constants.Shooter.FeederBackDistance);
    }
    
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
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return true;
  }
  @Override
  public InterruptionBehavior getInterruptionBehavior()
  {
    return Command.InterruptionBehavior.kCancelIncoming;
  }
}
