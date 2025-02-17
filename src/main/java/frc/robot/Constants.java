// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
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
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
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
      public static final double BottomPosition = 0; 
      public static final double First = 0;//FIXME
      public static final double Second = 20;//TODO
      public static final double Third = 30;//TODO
      public static final double Fourth = 40;//TODO
      public static final double elvHighest = 65; //max height 71.854980
  }
  public static class ElevatorCalibrations 
  {

    //All of the PID and Feedforward gains for the MotionMagic Motion profiler.
    public static final double kElevatorkG = 0;
    public static final double kElevatorkS = 0.25; // 0.145?
    public static final double kElevatorkV = 0.12; 
    public static final double kElevatorkA = 0.01;
    public static final double kElevatorkP = 4.8;
    public static final double kElevatorkD = 0.10000000149011612;
    public static final double kElevatorkI = 0.00;

    // Motion Magic Configs for the MotionMagicConfigs class for the Elevator
    public static final double kMaxSpeedMotionMagic = 2.0;//cruise velocity
    public static final double kMaxAccelerationMotionMagic = 4.0;//300
    public static final double kMaxElevatorCurrentPerMotor = 800;

  }
  public static final class Coral
  {
      //SHOOTER
      public static final int IntakeMotor_ID = 23;

  }
  public static final class Climber
  {   
      //Climber
      public static final int ClimberMotor1_ID = 27;
      public static final int GrabMotor_ID = 1;
      public static final double zeroDegrees = 150;
      public static final double stowPosition = 140;  //255; 
      public static final double grabPosition = 195;//310; this one
      public static final double climbPosition = 190; //305;
      public static final double MaxLow = 115; 
      public static final double MaxHigh = 205;
  }
  public static final class Limelight
  {
    public static final String FRONT = "limelight-mech";
  }
}