
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem.Color;

public class StateMachine extends SubsystemBase 
{
  private int scoreHeight = 1;
  private boolean scoreLeft = true;
  private boolean killCommands;
  private boolean runIntake;
  private boolean elevRaised;
  private boolean smartDrive = true;
  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private boolean readyToClimb = false;
  private boolean readyToScore = false;
  private boolean coralPartialAquired = false;
  private CANdleSystem candle;

  //Initialization

  public StateMachine(CANdleSystem candle){this.candle = candle;}

  //Score Height Methods
  public void setScoreHeight(int input) { scoreHeight = input; }
  public int getScoreHeight() { return scoreHeight; }

  // Score Side Methods
  public void setScoreLeft(boolean isLeft) { scoreLeft = isLeft; }
  public boolean isScoreLeft() { return scoreLeft; }
  public boolean isScoreRight() { return !scoreLeft; }

  //coral
  public void setCoralPartialAquired(boolean part){coralPartialAquired = part;}
  public boolean getCoralPartialAquired(){return coralPartialAquired;}
  
  // Kill Commands Methods
  public void setKillCommands(boolean killCommands) { this.killCommands = killCommands; }
  public boolean GetKillCommands() { return killCommands; }
  
  //Intake Methods
  public void setIntake(boolean runIntake) { this.runIntake = runIntake; }
  public Trigger runIntake() { return new Trigger(() -> runIntake); }
  public Command IntakeCMD(boolean intakeBool) { return this.runOnce(() -> this.setIntake(intakeBool)); }

  // Elevator Speed Control Mehods
  //public double getMaxSpeed() { return elevRaised ? maxSpeed / 8 : maxSpeed; }
  public double getMaxSpeed() { return maxSpeed; }

  public void setElevRaised(boolean input) { elevRaised = input; }
  public boolean isElevRaised() { return elevRaised; }

  //Drive Methods
  public void SetDriveToSmart() { smartDrive = true; }
  public void SetDriveToManual() { smartDrive = false; }
  public boolean IsDriveModeSmart() { return smartDrive; }

  //Ready to Score
  public void SetReadyToScore(boolean bool) { readyToScore = bool; }
  public boolean IsReadyToScore() 
  { 
    if (DriverStation.isAutonomousEnabled())
      return true;
    else if(readyToScore || !IsDriveModeSmart())
      return true; 
    else
      return false;
  }

  //ready to climb
  public boolean isReadyToClimb()
  {
    return readyToClimb;
  }
  public void setReadyToClimb(boolean ready)
  {
    readyToClimb = ready;
  }
  public void periodic()
  { 
    SmartDashboard.putBoolean("Smart", smartDrive);
    if (DriverStation.isAutonomous() && (DriverStation.getMatchTime() <= 15 && DriverStation.getMatchTime() > 0)) 
    {
      candle.SetLowLeft(Constants.Candle.purple);
      candle.SetMidLeft(Constants.Candle.purple);
      candle.SetHighLeft(Constants.Candle.purple);
      candle.SetLowRight(Constants.Candle.purple);
      candle.SetMidRight(Constants.Candle.purple);
      candle.SetHighRight(Constants.Candle.purple);
    }
    else if (DriverStation.isAutonomous() && (DriverStation.getMatchTime() <= Constants.Climber.MaxMatchTime
        && DriverStation.getMatchTime() > 0)) 
    {
      candle.SetLowLeft(Constants.Candle.yellow);
      candle.SetMidLeft(Constants.Candle.yellow);
      candle.SetHighLeft(Constants.Candle.yellow);
      candle.SetLowRight(Constants.Candle.yellow);
      candle.SetMidRight(Constants.Candle.yellow);
      candle.SetHighRight(Constants.Candle.yellow);
    }
    else if(DriverStation.isDisabled())
    {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
      {
        candle.SetLowLeft(Constants.Candle.blue);
        candle.SetMidLeft(Constants.Candle.blue);
        candle.SetHighLeft(Constants.Candle.blue);
        candle.SetLowRight(Constants.Candle.blue);
        candle.SetMidRight(Constants.Candle.blue);
        candle.SetHighRight(Constants.Candle.blue);
      }
      else
      {
        candle.SetLowLeft(Constants.Candle.red);
        candle.SetMidLeft(Constants.Candle.red);
        candle.SetHighLeft(Constants.Candle.red);
        candle.SetLowRight(Constants.Candle.red);
        candle.SetMidRight(Constants.Candle.red);
        candle.SetHighRight(Constants.Candle.red);    
      }
    }
    else if(IsDriveModeSmart())
    {
      if(LimelightHelpers.getTV(Constants.Limelight.LEFT))
      {
        setColorsRight(Constants.Candle.green);
      }
      else
      {
        setColorsRight(Constants.Candle.blue);
      }
      if(LimelightHelpers.getTV(Constants.Limelight.RIGHT))
      {
        setColorsLeft(Constants.Candle.green);
      }
      else
      {
        setColorsLeft(Constants.Candle.blue);
      }
    }
    else
    {
      setColorsRight(Constants.Candle.red);
      setColorsLeft(Constants.Candle.red);
    }
  }
  public void setColorsRight(Color color)
  {
    switch(scoreHeight) 
    {
      case 0:
        candle.SetLowRight(Constants.Candle.yellow);
        candle.SetMidRight(Constants.Candle.black);
        candle.SetHighRight(Constants.Candle.black);
        break;
      case 1:
        candle.SetLowRight(color);
        candle.SetMidRight(Constants.Candle.black);
        candle.SetHighRight(Constants.Candle.black);
        break;
      case 2:
        candle.SetLowRight(color);
        candle.SetMidRight(color);
        candle.SetHighRight(Constants.Candle.black);
        break;
      case 3:
        candle.SetLowRight(color);
        candle.SetMidRight(color);
        candle.SetHighRight(color);
        break;
    }
  }
  public void setColorsLeft(Color color)
  {
    switch(scoreHeight) 
    {
      case 0:
        candle.SetLowLeft(Constants.Candle.yellow);
        candle.SetMidLeft(Constants.Candle.black);
        candle.SetHighLeft(Constants.Candle.black);
        break;
      case 1:
        candle.SetLowLeft(color);
        candle.SetMidLeft(Constants.Candle.black);
        candle.SetHighLeft(Constants.Candle.black);
        break;
      case 2:
        candle.SetLowLeft(color);
        candle.SetMidLeft(color);
        candle.SetHighLeft(Constants.Candle.black);
        break;
      case 3:
        candle.SetLowLeft(color);
        candle.SetMidLeft(color);
        candle.SetHighLeft(color);
        break;
    }
  }
}
