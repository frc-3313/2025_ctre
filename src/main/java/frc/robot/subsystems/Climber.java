
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.StateMachine;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase 
{  

  //private TalonFX climberMotor1;//kraken
  private Servo grabMotor;//servo
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private final PIDController pid;
  private double minPowerAtExtended = 0.00;

  private double newTargetPosition;

  public Climber() 
  {
    //climberMotor1 = new TalonFX(Constants.Climber.ClimberMotor1_ID, Constants.CANIVORE);//kraken
    grabMotor = new Servo(Constants.Climber.GrabMotor_ID);//kraken
    var TalonFXConfiguration = new TalonFXConfiguration();
    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    // set slot 0 gains
    var slot0Configs = TalonFXConfiguration.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    //climberMotor1.getConfigurator().apply(TalonFXConfiguration);
    
    

  //  pidController.setFeedbackDevice(m_encoder);
    //newTargetPosition = climberMotor1.getPosition().getValueAsDouble();
    grabMotor.setAngle(0);

    //PID
    kP = 0.1; //how aggresive towards target
    kI = 0; //accumlation of past errors
    kD = 0.0001; //how rate of change responds
    kMaxOutput = 1; //max motor speed
    kMinOutput = -1; //max motor speed in oppisite direction 
    pid = new PIDController(kP, kI, kD);
    pid.setIntegratorRange(kMaxOutput, kMinOutput);
    pid.setTolerance(0);
    //SmartDashboard.putBoolean("Display Climber", displaySmartDashboard);
  }

  public void GoToHeight(double height)
  {
    newTargetPosition = height;
  }

  public void Grab()
  {
    grabMotor.setAngle(180);
  }

  public void Release()
  {
    grabMotor.setAngle(0);
  }
  
  @Override
  public void periodic() 
  {
    // read PID coefficients from SmartDashboard
    final DynamicMotionMagicVoltage m_request = 
          new DynamicMotionMagicVoltage(0, 80, 400, 4000);
    //climberMotor1.setControl(m_request.withPosition(newTargetPosition));
  
      SmartDashboard.putNumber("SetPoint", newTargetPosition);
     // SmartDashboard.putNumber("Climber Encoder", climberMotor1.getPosition().getValueAsDouble());


  }
  public void setMaxSpeeds(double speedUp, double speedDown)
  {
    pid.setIntegratorRange(speedUp, speedDown);
  }
  public boolean atSetpoint() {
    if((getDegrees() < newTargetPosition + 3) && (getDegrees() > newTargetPosition - 3))
      return true;
    else
      return false;
  }
  public double getFeedForward() {
    return Math.cos(Math.toRadians(getDegrees()))*getMinPower();
  }

  public double getMinPower() {
    return minPowerAtExtended;
  }
  public double getDegrees() {
    return 0;
    //climberMotor1.getPosition().getValueAsDouble();
  }
}
