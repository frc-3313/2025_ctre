
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
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
    if (DriverStation.getMatchTime() <= 15 && DriverStation.getMatchTime() > 0) 
    {
      candle.SetLow(Constants.Candle.purple);
      candle.SetMid(Constants.Candle.purple);
      candle.SetHigh(Constants.Candle.purple);
    }
    else if (DriverStation.getMatchTime() <= Constants.Climber.MaxMatchTime
        && DriverStation.getMatchTime() > 0) 
    {
      candle.SetLow(Constants.Candle.yellow);
      candle.SetMid(Constants.Candle.yellow);
      candle.SetHigh(Constants.Candle.yellow);
    }
    else if(DriverStation.isDisabled())
    {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
      {
        candle.SetLow(Constants.Candle.blue);
        candle.SetMid(Constants.Candle.blue);
        candle.SetHigh(Constants.Candle.blue);
      }
      else
      {
        candle.SetLow(Constants.Candle.red);
        candle.SetMid(Constants.Candle.red);
        candle.SetHigh(Constants.Candle.red);     
      }
    }
    else if(IsDriveModeSmart())
    {
      if((scoreLeft && LimelightHelpers.getTV(Constants.Limelight.RIGHT)) ||
         !scoreLeft && LimelightHelpers.getTV(Constants.Limelight.LEFT))
      {
        setColors(Constants.Candle.green);
      }
      else
      {
        setColors(Constants.Candle.blue);
      }
    }
    else
    {
      setColors(Constants.Candle.red);
    }
  }
  public void setColors(Color color)
  {
    switch(scoreHeight) 
    {
      case 1:
        candle.SetLow(color);
        candle.SetMid(Constants.Candle.black);
        candle.SetHigh(Constants.Candle.black);
        break;
      case 2:
        candle.SetLow(color);
        candle.SetMid(color);
        candle.SetHigh(Constants.Candle.black);
        break;
      case 3:
        candle.SetLow(color);
        candle.SetMid(color);
        candle.SetHigh(color);
        break;
    }
  }
}
