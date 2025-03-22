
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

public class StateMachine extends SubsystemBase 
{
  private int scoreHeight = 0 ;
  private boolean scoreLeft = true;
  private boolean killCommands;
  private boolean runIntake;
  private boolean elevRaised;
  private boolean smartDrive = true;
  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private boolean readyToClimb = false;
  public boolean hasCoral = false;

  //Initialization

  public StateMachine(){}

  //Score Height Methods
  public void setScoreHeight(int input) { scoreHeight = input; }
  public int getScoreHeight() { return scoreHeight; }

  // Score Side Methods
  public void setScoreLeft(boolean isLeft) { scoreLeft = isLeft; }
  public boolean isScoreLeft() { return scoreLeft; }
  public boolean isScoreRight() { return !scoreLeft; }
  
  // Kill Commands Methods
  public void setKillCommands(boolean killCommands) { this.killCommands = killCommands; }
  public boolean GetKillCommands() { return killCommands; }
  
  //Intake Methods
  public void setIntake(boolean runIntake) { this.runIntake = runIntake; }
  public Trigger runIntake() { return new Trigger(() -> runIntake); }
  public Command IntakeCMD(boolean intakeBool) { return this.runOnce(() -> this.setIntake(intakeBool)); }
  public boolean HasCoral(){return hasCoral;}

  // Elevator Speed Control Mehods
  //public double getMaxSpeed() { return elevRaised ? maxSpeed / 8 : maxSpeed; }
  public double getMaxSpeed() { return maxSpeed; }

  public void setElevRaised(boolean input) { elevRaised = input; }
  public boolean isElevRaised() { return elevRaised; }

  //Drive Methods
  public void SetDriveToSmart() { smartDrive = true; }
  public void SetDriveToManual() { smartDrive = false; }
  public boolean IsDriveModeSmart() { return smartDrive; }

  //ready to climb
  public boolean isReadyToClimb()
  {
    return readyToClimb;
  }
  public void setReadyToClimb(boolean ready)
  {
    readyToClimb = ready;
  }
}
