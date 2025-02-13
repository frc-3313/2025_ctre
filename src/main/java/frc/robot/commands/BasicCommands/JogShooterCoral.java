// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;


public class JogShooterCoral extends Command {
  public Shooter shooter;
  public Boolean inward;
  public Timer timer;
  /** Creates a new AmpScoreCMD. */
  public JogShooterCoral(Shooter m_shooter, Boolean m_inward){
    shooter = m_shooter;
    inward = m_inward;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  // tilter.GoToPosition(Constants.Tilter.ampPosition); 
  // shooter.StartShooter();
    timer = new Timer();
    timer.start();
    if(inward)
      shooter.StartFeeder(.1);
    else  
      shooter.StartFeeder(-.1);
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
    if(timer.hasElapsed(.1))
    {
      shooter.StopFeeder();
      return true;
    }
    else
      return false;
  }
}
