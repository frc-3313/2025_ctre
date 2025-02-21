// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends SubsystemBase {

  private final TalonFX masterMotor = new TalonFX(Constants.Elevator.ElevatorMotor1_ID, Constants.CANIVORE);
  private final TalonFX slaveMotor = new TalonFX(Constants.Elevator.ElevatorMotor2_ID, Constants.CANIVORE);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private double newTargetPosition = 0;
  public Elevator() 
  {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    
    // Configure PID values
    masterConfig.Slot0.kP = Constants.Elevator.kP;
    masterConfig.Slot0.kI = Constants.Elevator.kI;
    masterConfig.Slot0.kD = Constants.Elevator.kD;
    
    // Configure Motion Magic settings
    masterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.CRUISE_VELOCITY;
    masterConfig.MotionMagic.MotionMagicAcceleration = Constants.Elevator.ACCELERATION;
    
    // 🚨 Configure Soft Limits
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Elevator.MAX_HEIGHT;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;
    
    masterMotor.getConfigurator().apply(masterConfig);

    masterMotor.setNeutralMode(NeutralModeValue.Brake);
    slaveMotor.setNeutralMode(NeutralModeValue.Brake);

    slaveMotor.setControl(new Follower(Constants.Elevator.ElevatorMotor1_ID, true));
  }

  public void setHeight(double targetPos)
  {
    if(targetPos > Constants.Elevator.MAX_HEIGHT)
    {
      targetPos = Constants.Elevator.MAX_HEIGHT;
    }
    else if (targetPos < Constants.Elevator.MIN_HEIGHT) 
    {
      targetPos = Constants.Elevator.MIN_HEIGHT;      
    }
    newTargetPosition = targetPos;
    masterMotor.setControl(motionMagic.withPosition(newTargetPosition).withSlot(0).withIgnoreHardwareLimits(true).withOverrideBrakeDurNeutral(true));
  }

  public double getCurrentPosition()
  {
    return masterMotor.getPosition().getValueAsDouble();
  }

  public void stop()
  {
    masterMotor.stopMotor();
  }

  public boolean atSetpoint()
  {
    if(masterMotor.getPosition().getValueAsDouble() > newTargetPosition - 1 &&
    masterMotor.getPosition().getValueAsDouble() < newTargetPosition + 1 )
      return true;
    else 
      return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator set", m_request.Position);
    SmartDashboard.putNumber("Elevator current", getCurrentPosition());
    
  }

  @Override
  public void simulationPeriodic() {
  }
}