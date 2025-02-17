// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANdiConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Creates the TalonFX motors for the elevator.
  private final TalonFX m_elevator1;
  private final TalonFX m_elevator2;

  // Creates two followers for running the motors in sync.
  private final Follower m_followerInv;

  private TalonFXConfiguration m_config;

  private final MotionMagicVoltage m_request;

  public Elevator() {

    // Initializes the TalonFX motors for the elevator.
    m_elevator1 = new TalonFX(Constants.Elevator.ElevatorMotor1_ID, Constants.CANIVORE);
    m_elevator2 = new TalonFX(Constants.Elevator.ElevatorMotor2_ID, Constants.CANIVORE);

    final MotionMagicVoltage request = new MotionMagicVoltage(0);

    m_request = request;

    // Creates a configurator for the motors in this subsystem.
    TalonFXConfiguration config = new TalonFXConfiguration();

    Slot0Configs slot0Configs = config.Slot0;

    MotionMagicConfigs magicConfigs = config.MotionMagic;

    TorqueCurrentConfigs currentConfig = config.TorqueCurrent;

    SoftwareLimitSwitchConfigs softLimitConfigs = config.SoftwareLimitSwitch;

    HardwareLimitSwitchConfigs limitConfigs = config.HardwareLimitSwitch;

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

    //limitConfigs.ReverseLimitAutosetPositionEnable = m_CaNdi.getS1Closed().hasUpdated();
    //limitConfigs.ReverseLimitAutosetPositionValue = 0.0;

    // Configures all of the soft limit settings on the elevator1 motor
    softLimitConfigs.ForwardSoftLimitEnable = true;
    softLimitConfigs.ForwardSoftLimitThreshold = 87;
    
    // Applies the configs to all the motors in this subsystem.
    m_elevator1.getConfigurator().apply(config);  
    m_elevator2.getConfigurator().apply(config);

    // Sets the neutral mode of all of the elevator motors to Brake Mode.
    m_elevator1.setNeutralMode(NeutralModeValue.Brake);
    m_elevator2.setNeutralMode(NeutralModeValue.Brake);

    // Declares elevator1 as lead motor. Other motors are set to follow.
    m_followerInv = new Follower(Constants.Elevator.ElevatorMotor1_ID, true);

    // Setting the follower mode that each motor will follow.
    m_elevator2.setControl(m_followerInv);

    m_config = config;
  }

  /**
   * Passes in a value in degrees for the Motion Magic Motion Profiler to use.
   * 
   * @param newElevatorSetpoint - New setpoint for the elevator in inches.
   */
  public void GoToHeight(double newElevatorSetpoint) {

    //Sets the setpoint of elevator1 motor using the MotionMagic Motion Profiler.
    //m_elevator1.setControl(m_motionMagicTorqueCurrentFOC.withPosition(newElevatorSetpoint * Constants.ElevatorConstants.kPulleyGearRatio));
    m_elevator1.setControl(m_request.withPosition(newElevatorSetpoint));

  }

  /**
   * Passes in a value for manual control of the elevator as velocity.
   * 
   * @param newElevatorVelocity - Motor output from a scale of 0 to 1.
   */
  public void setElevatorVelocity(double newElevatorVelocity) {

    // Sets the elevator velocity on a scale from 0 to 1.
    m_elevator1.set(newElevatorVelocity);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

public void editConfig() {
  // m_config.Slot0.kS = SmartDashboard.getNumber("elevator kS", Calibrations.ElevatorCalibrations.kElevatorkS);
  // m_config.Slot0.kG = SmartDashboard.getNumber("elevator kG", Calibrations.ElevatorCalibrations.kElevatorkG);
  // m_config.Slot0.kP = SmartDashboard.getNumber("elevator kP", Calibrations.ElevatorCalibrations.kElevatorkP);
  // m_config.Slot0.kD = SmartDashboard.getNumber("elevator kD", Calibrations.ElevatorCalibrations.kElevatorkD);

  // m_elevator1.getConfigurator().apply(m_config);

}

public double getPosition() {
  return m_elevator1.getPosition().getValueAsDouble();
}


}