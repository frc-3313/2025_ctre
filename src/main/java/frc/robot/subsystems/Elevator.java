
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Elevator extends SubsystemBase {
  

  private TalonFX elevatorMotor1;//kraken
  private TalonFX elevatorMotor2;//kraken
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double minPowerAtExtended = 0.00;
  private final DynamicMotionMagicDutyCycle m_request = new DynamicMotionMagicDutyCycle(0, 5, 5, 1600);
  private double newTargetPosition;

  public Elevator() 
  {
    elevatorMotor1 = new TalonFX(Constants.Elevator.ElevatorMotor1_ID, Constants.CANIVORE);//kraken
    elevatorMotor2 = new TalonFX(Constants.Elevator.ElevatorMotor2_ID, Constants.CANIVORE);//kraken
    var TalonFXConfiguration = new TalonFXConfiguration();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    TalonFXConfiguration.MotorOutput = motorOutputConfigs;
    // set slot 0 gains
    var slot0Configs = TalonFXConfiguration.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = .001; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.0001; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 1; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 1; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    //elevatorMotor2.getConfigurator().apply(TalonFXConfiguration);
    //elevatorMotor1.getConfigurator().apply(TalonFXConfiguration);
    var follower = new Follower(elevatorMotor1.getDeviceID(), true);
    //elevatorMotor2.setControl(follower);

  //  pidController.setFeedbackDevice(m_encoder);
    newTargetPosition = elevatorMotor1.getPosition().getValueAsDouble();

  }

  public void GoToHeight(double height)
  {
    newTargetPosition = height;
  }

  
  @Override
  public void periodic() 
  {
    // read PID coefficients from SmartDashboard
    m_request.withPosition(newTargetPosition); 
    elevatorMotor1.setControl(m_request);
  
      SmartDashboard.putNumber("Elevator SetPoint", newTargetPosition);
      SmartDashboard.putNumber("Elevator Encoder", elevatorMotor1.getPosition().getValueAsDouble());


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
    return elevatorMotor1.getPosition().getValueAsDouble();
  }
}
