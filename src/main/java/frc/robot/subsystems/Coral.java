
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Coral extends SubsystemBase 
{

  //Intake Motor Setup
  private final TalonFX intakeMotor = new TalonFX(Constants.Coral.IntakeMotor_ID, Constants.CANIVORE);
  private DigitalInput coralPartiallyAcquired = new DigitalInput(0);
  private DigitalInput coralFullyAcquired = new DigitalInput(1);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);


  public Coral() 
  {
    var TalonFXConfiguration = new TalonFXConfiguration();
    // Configure PID values
    TalonFXConfiguration.Slot0.kP = Constants.Coral.kP;
    TalonFXConfiguration.Slot0.kI = Constants.Coral.kI;
    TalonFXConfiguration.Slot0.kD = Constants.Coral.kD;
    
    // Configure Motion Magic settings
    TalonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants.Coral.CRUISE_VELOCITY;
    TalonFXConfiguration.MotionMagic.MotionMagicAcceleration = Constants.Coral.ACCELERATION;
      

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    intakeMotor.getConfigurator().apply(TalonFXConfiguration);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  public void RunIntake(double speed)
  {
    intakeMotor.set(speed);
   
  }
  public void StopIntake()
  {
    intakeMotor.set(0);
    var targetPos = intakeMotor.getPosition().getValueAsDouble();
    intakeMotor.setControl(motionMagic.withPosition(targetPos).withSlot(0).withIgnoreHardwareLimits(true).withOverrideBrakeDurNeutral(true));

  }

  @Override
  public void periodic() 
  {
    
    if(intakeMotor.getVelocity().getValueAsDouble() > 10)
    {
      SmartDashboard.putBoolean("Intake running", true);
    }
    else
    {
      SmartDashboard.putBoolean("Intake running", false);
    }

    SmartDashboard.putBoolean("coral partially acquired", coralPartiallyAcquired());
    SmartDashboard.putBoolean("coral fully acquired", coralFullyAcquired());
    
  }

  public Boolean coralPartiallyAcquired()
   {
      return !coralPartiallyAcquired.get();
   }
   public boolean coralFullyAcquired()
  {
    return !coralFullyAcquired.get();
  }

}