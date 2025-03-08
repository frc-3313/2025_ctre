// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbGrabPositionCMD extends Command 
{

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
      climber.Release();;
      // elevator.setMotorAmp(80);
    //  climber.lower();
    //}

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
    return climber.atSetpoint();
    
  }
}
