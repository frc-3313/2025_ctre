// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;


public class ScoreCoralPrimeCMD extends InstantCommand {

  public Coral coral;
  public Elevator elevator;
  /*what should replace this? Right or left? Which height it should go to?*/private Double shootAngle;
  private Double elevatorHeight;//how does this work with this robot?
  private double shootSpeed;//is this still necessary for this robot?
  
  public ScoreCoralPrimeCMD(Coral coral, Elevator elevator, Double shootSpeed, Double shootAngle, Double elevatorHeight) 
  {
    this.coral = coral;
    this.elevator = elevator;
    this.shootAngle = shootAngle;//right or left? which height?
    this.elevatorHeight = elevatorHeight;//how does this work?
    this.shootSpeed = shootSpeed;//is this necessary?
    addRequirements(coral, elevator);
  }

  @Override
  public void initialize() 
  {
    coral.SetShooterSpeed(shootSpeed);//this should be pretty similar
    elevator.GoToHeight(elevatorHeight);
    
  }
}
