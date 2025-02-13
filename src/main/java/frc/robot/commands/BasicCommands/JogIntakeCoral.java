// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;


public class JogIntakeCoral extends Command {
  public Intake intake;
  public Boolean inward;
  public Timer timer;
  /** Creates a new AmpScoreCMD. */
  public JogIntakeCoral(Intake m_Intake, Boolean m_inward){
    intake = m_Intake;
    inward = m_inward;
    addRequirements(intake);
    
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
      intake.RunIntake(.3);
    else  
      intake.RunIntake(-.3);
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
    intake.StopIntake();
    
  // shooter.StopAllMotors(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

      return false;
  }
}
