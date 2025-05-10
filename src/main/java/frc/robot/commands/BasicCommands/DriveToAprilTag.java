// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class DriveToAprilTag extends Command {

  private final CommandSwerveDrivetrain swerveDrive;
  private final StateMachine stateMachine;
  private String limelight;

  // private final SwerveRequest.FieldCentricFacingAngle driveRequest = new FieldCentricFacingAngle()
  // .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotController;

  // private double offsetRightX = 0.04599; //0.054
  // private double offsetRightY = -0.061; //-0.001
  // private double offsetLeftX = 0.002; //0.039
  // private double offsetLeftY = -0.032; //0.0005
  private double offsetRightX = 0.01495285052806139; //0.0430; //0.1905
  private double offsetRightY = -0.020561281591653824;//-0.049; //-0.051
  private double offsetLeftX = -0.0034890174865722656;//0.0238; //-0.017
  private double offsetLeftY = -0.05534464493393898;//0.0125; //-0.014
  private double offsetRightLevel1X = 0.5957528948783875;
  private double offsetRightLevel1Y = -0.010251236148178577;
  private double offsetLeftLevel1X = -0.614608883857727;
  private double offsetLeftLevel1Y = -0.0038976033683866262;
  private double Xkp = .11; //.11
  private double Xkd = .003;
  private double ykp = .07; //.11
  private double ykd = .003;
  private double rotkp = .09; //.11
  private double rotkd = .00;
  private double offsetX, offsetY;

  private double txError = 0.55, tyError = 0.4;

  public DriveToAprilTag(CommandSwerveDrivetrain swerveDrive, StateMachine stateMachine) {
    this.swerveDrive = swerveDrive;
    this.stateMachine = stateMachine;
    this.xController = new PIDController(Xkp, 0.0, Xkd);
    this.yController = new PIDController(ykp, 0.0, ykd);
    this.rotController = new PIDController(rotkp, 0.0, rotkd);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    stateMachine.SetReadyToScore(false);

    // Reset PID controllers
    xController.reset();
    yController.reset();
    rotController.reset();
    if(stateMachine.isScoreLeft())
    {
        limelight = Constants.Limelight.RIGHT;
        LimelightHelpers.setFiducial3DOffset(Constants.Limelight.RIGHT, offsetRightX, offsetRightY, 0);
        offsetX = offsetRightX;
        offsetY = offsetRightY;

    }
    else{
      limelight = Constants.Limelight.LEFT;
      LimelightHelpers.setFiducial3DOffset(Constants.Limelight.LEFT, offsetLeftX, offsetLeftY, 0);
      offsetX = offsetLeftX;
      offsetY = offsetLeftY;
    }
    // driveRequest.HeadingController = new PhoenixPIDController(4, 0, 0);
    // driveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    
  }

  @Override
  public void execute()
  {
    double tx = LimelightHelpers.getTX(limelight); // Horizontal offset to offset POI
    double ty = LimelightHelpers.getTY(limelight); // Vertical offset to offset POI
    double positions[] = LimelightHelpers.getBotPose_TargetSpace(limelight);
    double rot = positions[4];
    boolean tv = LimelightHelpers.getTV(limelight);

    tx += offsetX;
    ty += offsetY;

    if(tv)
    {
      double robotYVelocity = yController.calculate(tx, 0);
      double robotXVelocity = xController.calculate(ty, 0);
      double rotbotRot = rotController.calculate(rot, 0);

      driveRequest.VelocityY = robotYVelocity;
      driveRequest.VelocityX = -robotXVelocity;
      driveRequest.RotationalRate = -rotbotRot;

      swerveDrive.setControl(driveRequest);
    }

    SmartDashboard.putBoolean("drive to apirl rot", rotController.atSetpoint());
    SmartDashboard.putBoolean("drive to apirl ycont", rotController.atSetpoint());
    SmartDashboard.putBoolean("drive to apirl xcont", rotController.atSetpoint());

  }


  @Override
  public void end(boolean interrupted) {
    swerveDrive.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    swerveDrive.setControl(driveRequest
      .withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    if(LimelightHelpers.getTV(limelight))
    {
      if(Math.abs(LimelightHelpers.getTX(limelight)) <= txError && 
      LimelightHelpers.getTY(limelight) <= tyError)
      {
        stateMachine.SetReadyToScore(true);
        return true;
      }
    }
    return false;
  }
}
