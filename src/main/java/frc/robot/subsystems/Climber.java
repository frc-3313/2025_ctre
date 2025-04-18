// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.StateMachine;

public class Climber extends SubsystemBase {

  private final TalonFX masterMotor = new TalonFX(Constants.Climber.ClimberMotor1_ID, Constants.CANIVORE);

  private double newTargetPosition = 0;
  private final StateMachine stateMachine;
  private Servo grabMotor;//servo
  private Servo leftMotor;//servo
  private Servo rightMotor;//servo
  private DutyCycleEncoder thruBore;

  public Climber(StateMachine _stateMachine) 
  {
    //26//45//147
    stateMachine = _stateMachine;
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    grabMotor = new Servo(Constants.Climber.GrabMotor_ID);//kraken
    leftMotor = new Servo(Constants.Climber.LeftMotor_ID);//kraken
    rightMotor = new Servo(Constants.Climber.RightMotor_ID);//kraken
    thruBore = new DutyCycleEncoder(2,360,320);
    // Configure PID values
    masterConfig.Slot0.kP = Constants.Climber.kP;
    masterConfig.Slot0.kI = Constants.Climber.kI;
    masterConfig.Slot0.kD = Constants.Climber.kD;
    // Configure Motion Magic settings
    masterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.CRUISE_VELOCITY;
    masterConfig.MotionMagic.MotionMagicAcceleration = Constants.Climber.ACCELERATION;
    masterConfig.TorqueCurrent.PeakReverseTorqueCurrent = 800;  
    
    masterMotor.getConfigurator().apply(masterConfig);

    masterMotor.setNeutralMode(NeutralModeValue.Brake);
    masterMotor.setPosition(320);
  }

  public void setSpeed(double speed, double pos)
  {
    newTargetPosition = pos;
    masterMotor.set(-speed);
  }
  public void Release()
  {
    grabMotor.setAngle(90);
  }
  public void ReleaseWings()
  {
    leftMotor.setSpeed(-50);
    rightMotor.setSpeed(50);
  }
  public void stopWings()
  {
    leftMotor.setDisabled();
    rightMotor.setDisabled();
  }
  public void Lock()
  {
    grabMotor.setAngle(0);
  }
  public double getCurrentPosition()
  {
    return masterMotor.getPosition().getValueAsDouble();
  }

  public void stop()
  {
    masterMotor.stopMotor();
  }

  public double getServoPos()
  {
    return grabMotor.getAngle();
  }
  public boolean atSetpoint()
  {
    if(thruBore.get() > newTargetPosition - .1 &&
    thruBore.get() < newTargetPosition + .1 )
      return true;
    else 
      return false;
  }

  @Override
  public void periodic() 
  {
  }
  public double getEncoder()
  {
    return thruBore.get();
  }
  @Override
  public void simulationPeriodic() {
  }
}