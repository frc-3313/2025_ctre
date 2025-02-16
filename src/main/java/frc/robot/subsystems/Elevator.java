
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  

  private final TalonFX elevatorMotor1;//kraken
  private final TalonFX elevatorMotor2;//kraken
  private double minPowerAtExtended = 0.00;
  private final MotionMagicVoltage  m_request;
  private double newTargetPosition;
  private final Follower m_followerInv;
  private final MotionMagicTorqueCurrentFOC m_motionMagicTorqueCurrentFOC;


  public Elevator() 
  {
    elevatorMotor1 = new TalonFX(Constants.Elevator.ElevatorMotor1_ID, Constants.CANIVORE);//kraken
    elevatorMotor2 = new TalonFX(Constants.Elevator.ElevatorMotor2_ID, Constants.CANIVORE);//kraken

    // initializes the motion magic motion profiler.
    m_motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);

    final MotionMagicVoltage request = new MotionMagicVoltage(0);

    m_request = request;

    var TalonFXConfiguration = new TalonFXConfiguration();

    // set slot 0 gains
    Slot0Configs slot0Configs = TalonFXConfiguration.Slot0;

    MotionMagicConfigs magicConfigs = TalonFXConfiguration.MotionMagic;

    TorqueCurrentConfigs currentConfig = TalonFXConfiguration.TorqueCurrent;

    SoftwareLimitSwitchConfigs softLimitConfigs = TalonFXConfiguration.SoftwareLimitSwitch;

    // Gravity type for this subsystem.
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        
    // Feedforward and PID settings for the motors in this subsystem.
    slot0Configs.kG = Constants.ElevatorCalibrations.kElevatorkG;
    slot0Configs.kS = Constants.ElevatorCalibrations.kElevatorkS;
    slot0Configs.kV = Constants.ElevatorCalibrations.kElevatorkV;
    slot0Configs.kA = Constants.ElevatorCalibrations.kElevatorkA;
    slot0Configs.kP = Constants.ElevatorCalibrations.kElevatorkP;
    slot0Configs.kD = Constants.ElevatorCalibrations.kElevatorkD;

    // Configs to be used by the MotionMagicConfigs class
    magicConfigs.MotionMagicCruiseVelocity = Constants.ElevatorCalibrations.kMaxSpeedMotionMagic;
    magicConfigs.MotionMagicAcceleration = Constants.ElevatorCalibrations.kMaxAccelerationMotionMagic;
    currentConfig.PeakForwardTorqueCurrent = Constants.ElevatorCalibrations.kMaxElevatorCurrentPerMotor;
    currentConfig.PeakReverseTorqueCurrent = -Constants.ElevatorCalibrations.kMaxElevatorCurrentPerMotor;

    // Configures all of the soft limit settings on the elevator1 motor
    softLimitConfigs.ForwardSoftLimitEnable = true;
    softLimitConfigs.ForwardSoftLimitThreshold = 87;

    // Applies the configs to all the motors in this subsystem.
    elevatorMotor1.getConfigurator().apply(TalonFXConfiguration);  
    elevatorMotor2.getConfigurator().apply(TalonFXConfiguration);
    // Declares elevator1 as lead motor. Other motors are set to follow.
    m_followerInv = new Follower(Constants.Elevator.ElevatorMotor1_ID, true);

    // Setting the follower mode that each motor will follow.
    elevatorMotor2.setControl(m_followerInv);
  }

  public void GoToHeight(double height)
  {
    newTargetPosition = height;
    elevatorMotor1.setControl(m_request.withPosition(newTargetPosition));

  }

  
  @Override
  public void periodic() 
  {
    // read PID coefficients from SmartDashboard
  
      SmartDashboard.putNumber("Elevator SetPoint", newTargetPosition);
      SmartDashboard.putNumber("Elevator Encoder", elevatorMotor1.getPosition().getValueAsDouble());
      SmartDashboard.putData("Arm PID", elevatorMotor1);


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
