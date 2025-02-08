// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Elevator;


public class PrimeScoreCoralCMD extends InstantCommand {

  public Tilter tilter;
  public Shooter shooter;
  public Elevator elevator;
  private Double shootAngle;
  private Double elevatorHeight;
  private double shootSpeed;
  
  public PrimeScoreCoralCMD(Tilter tilter, Shooter shooter, Elevator elevator, Double shootSpeed, Double shootAngle, Double elevatorHeight) 
  {
    this.tilter = tilter;
    this.shooter = shooter;
    this.elevator = elevator;
    this.shootAngle = shootAngle;
    this.elevatorHeight = elevatorHeight;
    this.shootSpeed = shootSpeed;
    addRequirements(tilter, shooter, elevator);
  }

  @Override
  public void initialize() 
  {
    shooter.SetShooterSpeed(shootSpeed);
    elevator.GoToHeight(elevatorHeight);
    tilter.GoToPosition(shootAngle);
    
  }
}
