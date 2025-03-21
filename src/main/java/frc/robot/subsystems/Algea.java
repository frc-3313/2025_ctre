
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.StateMachine;

public class Algea extends SubsystemBase 
{

  //Intake Motor Setup
 // private final TalonFX intakeMotor = new TalonFX(Constants.Algea.IntakeMotor_ID, Constants.CANIVORE);
  private DigitalInput algeaAcquired = new DigitalInput(2);
  private StateMachine stateMachine;
  public Algea(StateMachine stateMachine) 
  {
    this.stateMachine = stateMachine;
    // var TalonFXConfiguration = new TalonFXConfiguration();
    // var motorConfigs = new MotorOutputConfigs();
    // motorConfigs.NeutralMode = NeutralModeValue.Brake;
    //intakeMotor.getConfigurator().apply(TalonFXConfiguration);
    

  }

  public void RunIntake(double speed)
  {
   // intakeMotor.set(speed);
   
  }
  public void StopIntake()
  {
    //intakeMotor.set(0);
  }


  @Override
  public void periodic() 
  {
    
    // if(intakeMotor.getVelocity().getValueAsDouble() > 10)
    // {
    //   SmartDashboard.putBoolean("Intake running", true);
    // }
    // else
    // {
    //   SmartDashboard.putBoolean("Intake running", false);
    // }

    // SmartDashboard.putBoolean("algea acquired", algeaAcquired());
    
  }

  public Boolean algeaAcquired()
   {
      return !algeaAcquired.get();
   }
   
  
}
