// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.SignalLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
// Set the logger to log to the first flashdrive plugged in

// Explicitly start the logger
 public Robot() {
  
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.setPath("/media/sda1/");
    SignalLogger.start();
    m_robotContainer.drivetrain.zeroGyroAuto();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putData(CommandScheduler.getInstance());
    LimelightHelpers.setLEDMode_ForceOn(Constants.Limelight.LEFT);
    LimelightHelpers.setLEDMode_ForceOn(Constants.Limelight.RIGHT);

  }

  @Override
  public void disabledInit() {}
    
  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    //m_autonomousCommand.schedule();

    //m_robotContainer.drivetrain.zeroGyro();

    // m_robotContainer.setAlgaeStow();
    if (m_autonomousCommand != null) 
    {
      SequentialCommandGroup autoCommand;
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
      {                
        autoCommand =  m_robotContainer.drivetrain.updateVisionCommand().andThen(new WaitCommand(.2)).andThen(m_autonomousCommand);
      }
      else
      {
        autoCommand = m_robotContainer.drivetrain.updateVisionCommand().andThen(new WaitCommand(.2)).andThen(m_autonomousCommand);
      }
    autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() 
  {
    // m_robotContainer.setAlgaeStow();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() 
  {

  }

  @Override
  public void teleopExit()
  {
    SignalLogger.stop();
  }
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
