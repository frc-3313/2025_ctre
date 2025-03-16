// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public final class Constants
{

  public static final String CANIVORE = "canivore1";

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  /*public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }*/

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    public static final double KPDrive = 10;
    public static final double KPSteer = 7;
  }

  public static class OperatorConstants
  {

    // Joystick 
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  //// SUBSYSTEM CONSTANTS ////
  public static final class Algea
  {
      //INTAKE
      public static final int IntakeMotor_ID = 26;
      
      DigitalInput IntakeBeam = new DigitalInput(0);
  }   
  public static final class Elevator
  {
      //ELEVATOR
      public static final int ElevatorMotor1_ID = 27;
      public static final int ElevatorMotor2_ID = 28;
      public static final double BottomPosition = .65;
      public static final double First = 10;//FIXME
      public static final double Second = 20.3;//TODO
      public static final double Third = 40.3;//TODO
      public static final double Fourth = 71;//TODO
      public static final double elvHighest = 71.2; //max height 71.854980

      public static final int MASTER_MOTOR_ID = 10; // Update with actual CAN ID
      public static final int SLAVE_MOTOR_ID = 11;  // Update with actual CAN ID

      //PID & Motion Magic Constants
      public static final double kP = 4.8;
      public static final double kI = 0.00001;
      public static final double kD = 0.1;

      public static final double CRUISE_VELOCITY = 2000;  // Units per 100ms
      public static final double ACCELERATION = 480;     // Units per 100ms²

      //Soft Limits
      public static final double MIN_HEIGHT = 0.0;
      public static final double MAX_HEIGHT = 30000.0;
  }
  
  public static final class Coral
  {
      //SHOOTER
      public static final int IntakeMotor_ID = 23;
      //PID & Motion Magic Constants
      public static final double kP = 4.8;
      public static final double kI = 0.00001;
      public static final double kD = 0.1;

      public static final double CRUISE_VELOCITY = 2000;  // Units per 100ms
      public static final double ACCELERATION = 480;     // Units per 100ms²

      public static final double blueReefX = 4.797; //meters
      public static final double blueReefY = 4.386; //meters
      public static final double scoreRadius = 1.6256; //meters
      public static final double angleoffset = 5; //degrees

      //Soft Limits
      public static final double MIN_HEIGHT = 0.0;
      public static final double MAX_HEIGHT = 30000.0;

  }
  public static final class Climber
  {   
      public static final double MaxMatchTime = 30;
      //Climber
      public static final int ClimberMotor1_ID = 29;
      public static final int GrabMotor_ID = 0;
      public static final double RAISE = 285; 
      public static final double LOWER = 438;
      
      //PID & Motion Magic Constants
      public static final double kP = 10;
      public static final double kI = 0.00000;
      public static final double kD = 0.0;

      public static final double CRUISE_VELOCITY = 1500;  // Units per 100ms
      public static final double ACCELERATION = 480;     // Units per 100ms²

      //Soft Limits
      public static final double MIN_HEIGHT = 0.0;
      public static final double MAX_HEIGHT = 30000.0;
  }
  public static final class Limelight
  {
    public static final String FRONT = "limelight-mech";
    public static final String RIGHT = "limelight-right";

  }
}