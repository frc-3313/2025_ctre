// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.opencv.ml.StatModel;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.BasicCommands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * Constants.OperatorConstants.LEFT_X_DEADBAND).withRotationalDeadband(MaxAngularRate * Constants.OperatorConstants.LEFT_X_DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final StateMachine stateMachine = new StateMachine();
    private final Coral coral = new Coral();
    private final Elevator elevator = new Elevator(stateMachine);
    private final Algea algea = new Algea();
    private final Climber climber = new Climber();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipulator = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(drivetrain.getDriveRange(driveController.getLeftY()) * stateMachine.getMaxSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(drivetrain.getDriveRange(driveController.getLeftX())  * stateMachine.getMaxSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(drivetrain.getDriveRange(-driveController.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        //------------------------------Both Drive Modes---------------------------------------//
            //Driver
        //driveController.rightBumper().onTrue(new ClimbGrabPositionCMD(climber, MaxAngularRate));
        //driveController.rightTrigger().onTrue(new ClimbCMD(climber, MaxAngularRate));
        
            //Manipulator
        manipulator.rightBumper().onTrue(
            new InstantCommand(() -> stateMachine.setScoreLeft(false)));
        manipulator.leftBumper().onTrue(
            new InstantCommand(() -> stateMachine.setScoreLeft(true)));
        
        manipulator.povDown().onTrue(
            new InstantCommand(() -> stateMachine.setScoreHeight(1)));
        manipulator.povRight().onTrue(
            new InstantCommand(() -> stateMachine.setScoreHeight(2)));
        manipulator.povUp().onTrue(
            new InstantCommand(() -> stateMachine.setScoreHeight(3)));

            
        //------------------------------Change Drive Mode--------------------------------//
        driveController.povLeft().onTrue(
            new InstantCommand(() -> stateMachine.SetDriveToSmart()));
        driveController.povRight().onTrue(
            new InstantCommand(() -> stateMachine.SetDriveToManual()));
        
        if(stateMachine.IsDriveModeSmart()) //Smart Drive Mode
        {
            manipulator.a().onTrue(new SmartIntake(stateMachine, coral, drivetrain, driveController));
            manipulator.rightTrigger().onTrue(new CoralScoreDrive(stateMachine, drivetrain, driveController));
            manipulator.rightTrigger().onFalse(new SequentialCommandGroup(
                new GoToScoringPosition(drivetrain),
                new ScoreCoralHeightCMD(coral, elevator, stateMachine),
                new ScoreCoralCMD(coral, elevator, stateMachine)
            ));
            // manipulator.a().onTrue(new SmartIntake(stateMachine, coral, drivetrain, driveController));
            // manipulator.rightTrigger().onTrue(new CoralScoreDrive(stateMachine, drivetrain, driveController));
            // manipulator.rightTrigger().onFalse(new SequentialCommandGroup(
            //     new ParallelCommandGroup(
            //         new GoToScoringPosition(drivetrain),
            //         new ScoreCoralHeightCMD(coral, elevator, stateMachine)
            //     ),
            //     new ScoreCoralCMD(coral, elevator, stateMachine)
            // ));
        }
        else //Manual Drive Mode
        {
                //Manipulator
            manipulator.a().onTrue(new CoralCMD(coral, stateMachine, .3));
            manipulator.rightBumper().onTrue(new ScoreCoralHeightCMD(coral, elevator, stateMachine));
            manipulator.rightTrigger().onTrue(new ScoreCoralCMD(coral, elevator, stateMachine));
            manipulator.x().onTrue(new ReturnToNormal(coral, elevator, algea));
        }
     }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
