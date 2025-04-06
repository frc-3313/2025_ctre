// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.StateMachine;

public class Algae extends SubsystemBase {

  private final TalonFX tilterMoter = new TalonFX(Constants.Algae.TilterMotor_ID, Constants.CANIVORE);
  private final MotionMagicVoltage IntakeMagic = new MotionMagicVoltage(0);
  private final MotionMagicVelocityDutyCycle IntakeMagic2 = new MotionMagicVelocityDutyCycle(0);
 private final TalonFX intakeMotor = new TalonFX(Constants.Algae.IntakeMotor_ID, Constants.CANIVORE);

  private double newTargetPosition = 0;
  private final StateMachine stateMachine;
  private DutyCycleEncoder thruBore;
  private DigitalInput AlgaeAquired = new DigitalInput(4);
  private final PIDController tilterPidController;

  public Algae(StateMachine _stateMachine) 
  {
    tilterPidController = new PIDController(Constants.Algae.TilterkP, Constants.Algae.TilterkI, Constants.Algae.TilterkD);
    //26//45//147
    stateMachine = _stateMachine;
    
    thruBore = new DutyCycleEncoder(3,360,320);
    newTargetPosition = getEncoder();
    TalonFXConfiguration TilterConfig = new TalonFXConfiguration();
    // Configure PID values
    TilterConfig.Slot0.kP = Constants.Algae.TilterkP;
    TilterConfig.Slot0.kI = Constants.Algae.TilterkI;
    TilterConfig.Slot0.kD = Constants.Algae.TilterkD;
    // Configure Motion Magic settings
    TilterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Algae.TilterCRUISE_VELOCITY;
    TilterConfig.MotionMagic.MotionMagicAcceleration = Constants.Algae.TilterACCELERATION;
    TilterConfig.TorqueCurrent.PeakReverseTorqueCurrent = 800;  
    tilterMoter.getConfigurator().apply(TilterConfig);

    tilterMoter.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration IntakeConfig = new TalonFXConfiguration();
    // Configure PID values
    IntakeConfig.Slot0.kP = Constants.Algae.IntakekP;
    IntakeConfig.Slot0.kI = Constants.Algae.IntakekI;
    IntakeConfig.Slot0.kD = Constants.Algae.IntakekD;
    // Configure Motion Magic settings
    IntakeConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Algae.IntakeCRUISE_VELOCITY;
    IntakeConfig.MotionMagic.MotionMagicAcceleration = Constants.Algae.IntakeACCELERATION;
    IntakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = 800;  
    
    intakeMotor.getConfigurator().apply(IntakeConfig);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  public void setPos(double pos)
  {
    newTargetPosition = pos;
    tilterMoter.set(getPidOutput());

  }
  public boolean tilterAtSetpoint()
  {
    if(thruBore.get() > newTargetPosition - .1 &&
    thruBore.get() < newTargetPosition + .1 )
      return true;
    else 
      return false;
  }

  public Boolean AlgaeAcquired()
  {
     return !AlgaeAquired.get();
  }

  public void RunIntake(double speed)
  {
    intakeMotor.setControl(IntakeMagic2.withVelocity(speed).withSlot(0));
   
  }
  public void StopIntake()
  {
    var targetPos = intakeMotor.getPosition().getValueAsDouble();
    intakeMotor.setControl(IntakeMagic.withPosition(targetPos).withSlot(0).withIgnoreHardwareLimits(true).withOverrideBrakeDurNeutral(true));

  }
  @Override
  public void periodic() 
  {

    tilterMoter.set(getPidOutput());

    SmartDashboard.putNumber("Algae set", newTargetPosition);
    SmartDashboard.putNumber("Algae encoder", thruBore.get());
    SmartDashboard.putBoolean("AlgaeAquired", AlgaeAcquired());

  }
  public double getEncoder()
  {
    return thruBore.get();
  }
  private double getPidOutput()
  {
    tilterPidController.setD(stateMachine.getKd());
    tilterPidController.setD(stateMachine.getKi());
    tilterPidController.setD(stateMachine.getKp());
    return MathUtil.clamp(tilterPidController.calculate(getEncoder(), newTargetPosition), -0.05, 0.05) * -1;

  }
}