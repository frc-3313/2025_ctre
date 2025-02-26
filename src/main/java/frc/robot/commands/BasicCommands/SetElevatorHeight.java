// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;


public class SetElevatorHeight extends Command {

  private final Elevator elevator;
  private final double targetHeight;

  public SetElevatorHeight(Elevator elevator, double targetHeight) {
    this.elevator = elevator;
    this.targetHeight = targetHeight;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setHeight(targetHeight);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(elevator.atSetpoint())
      return true;
    else  
      return false;
  }
}
