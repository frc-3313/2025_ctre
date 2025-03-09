// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbGrabPositionCMD extends Command 
{
  public Timer timer;
  Climber climber;
  double position;
  public ClimbGrabPositionCMD(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
   // if (DriverStation.getMatchTime() <= 15.0) 
   // {
    timer = new Timer();
    timer.reset();
    timer.start();
    climber.Release();
    climber.Motor_Release();
      // elevator.setMotorAmp(80);
    //}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(.5)) {
      climber.lower();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    /*climber.atSetpoint();
    climber.setMotorBrake();*/
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //  if (DriverStation.getMatchTime() > 15.0) 
  //  {
  //    return true;
  //  }
      if (timer.hasElapsed(.6)) 
      {
        return climber.atSetpoint();
      }
      return false;
    
  }
}
