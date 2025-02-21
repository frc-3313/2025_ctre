// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algea;
import frc.robot.subsystems.Coral;

public class ReturnToNormal extends InstantCommand {
  public Algea algea;
  public Elevator elevator;
  public Coral coral;
  public Timer timer;

  /** Creates a new AmpScoreCMD. */
  public ReturnToNormal(Coral m_Coral, Elevator m_Elevator, Algea m_Algea){
    algea = m_Algea;
    elevator = m_Elevator;
    coral = m_Coral;
    addRequirements(algea, elevator, coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    coral.StopIntake();
    algea.StopIntake();
    elevator.setHeight(Constants.Elevator.BottomPosition);
  }
}
