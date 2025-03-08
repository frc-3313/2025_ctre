// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.StateMachine;

public class Climber extends SubsystemBase {

  private final TalonFX masterMotor = new TalonFX(Constants.Climber.ClimberMotor1_ID, Constants.CANIVORE);
 private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private double newTargetPosition = 0;
  private final StateMachine stateMachine;
  private Servo grabMotor;//servo

  public Climber(StateMachine _stateMachine) 
  {
    stateMachine = _stateMachine;
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    grabMotor = new Servo(Constants.Climber.GrabMotor_ID);//kraken

    // Configure PID values
    masterConfig.Slot0.kP = Constants.Climber.kP;
    masterConfig.Slot0.kI = Constants.Climber.kI;
    masterConfig.Slot0.kD = Constants.Climber.kD;
    
    // Configure Motion Magic settings
    masterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.CRUISE_VELOCITY;
    masterConfig.MotionMagic.MotionMagicAcceleration = Constants.Climber.ACCELERATION;
    
    // 🚨 Configure Soft Limits
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.MAX_HEIGHT;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;
    
    masterMotor.getConfigurator().apply(masterConfig);

    masterMotor.setNeutralMode(NeutralModeValue.Brake);
    masterMotor.setPosition(0);
  }

  public void Raise()
  {
    newTargetPosition = Constants.Climber.RAISE;
    masterMotor.setControl(motionMagic.withPosition(newTargetPosition).withSlot(0).withIgnoreHardwareLimits(true).withOverrideBrakeDurNeutral(true));
  }
  public void lower()
  {
    newTargetPosition = Constants.Climber.LOWER;
    masterMotor.setControl(motionMagic.withPosition(newTargetPosition).withSlot(0).withIgnoreHardwareLimits(true).withOverrideBrakeDurNeutral(true));

  }
  public void Motor_Release()
  {
    newTargetPosition = masterMotor.getPosition().getValueAsDouble() - 1;
    masterMotor.setControl(motionMagic.withPosition(newTargetPosition).withSlot(0).withIgnoreHardwareLimits(true).withOverrideBrakeDurNeutral(true));
  }
  public void Release()
  {
    grabMotor.setAngle(90);
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
    if(masterMotor.getPosition().getValueAsDouble() > newTargetPosition - .1 &&
    masterMotor.getPosition().getValueAsDouble() < newTargetPosition + .1 )
      return true;
    else 
      return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber set", m_request.Position);
    SmartDashboard.putNumber("Climber current", getCurrentPosition());
    SmartDashboard.putNumber("servo current", getServoPos());

  }

  @Override
  public void simulationPeriodic() {
  }
}