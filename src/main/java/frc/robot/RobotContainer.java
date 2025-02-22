// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Coral coral = new Coral();
    private final Elevator elevator = new Elevator();
    private final StateMachine stateMachine = new StateMachine();
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
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


        //commands for manipulator
        manipulator.a().onTrue(new CoralCMD(coral, stateMachine));
        manipulator.rightTrigger().onTrue(new ScoreCoralCMD(coral, elevator, stateMachine));
        //manipulator.b().onTrue(new ScoreAlgeaCMD(algea, .5));
        //manipulator.y().onTrue(new ScoreAlgeaCMD(algea, -.5));
        manipulator.x().onTrue(new ReturnToNormal(coral, elevator, algea));
        manipulator.povDown().onTrue(new SetScoreHeightCMD(stateMachine, 0));
        manipulator.povLeft().onTrue(new SetScoreHeightCMD(stateMachine, 1));
        manipulator.povRight().onTrue(new SetScoreHeightCMD(stateMachine, 2));
        manipulator.povUp().onTrue(new SetScoreHeightCMD(stateMachine, 3));

        //FIX ME

        /* switched manipulator dpad from direct to height to state machine

        manipulator.povUp().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.Fourth));
        manipulator.povDown().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.BottomPosition));
        manipulator.povRight().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.Second));
        manipulator.povLeft().onTrue(new SetElevatorHeight(elevator, Constants.Elevator.Third));
        */


        //commands for driver
        //driveController.rightBumper().onTrue(new ClimbGrabPositionCMD(climber, MaxAngularRate));
        //driveController.rightTrigger().onTrue(new ClimbCMD(climber, MaxAngularRate));
        driveController.rightStick().onTrue(new SetScoreLeftCMD(stateMachine, true));
        driveController.leftStick().onTrue(new SetScoreRightCMD(stateMachine, true));
        driveController.start().onTrue(new ZeroGyro(drivetrain));
        driveController.a().onTrue(new RotateToAngleCMD(drivetrain, 0));
        driveController.b().onTrue(new RotateToAngleCMD(drivetrain, 60));
        driveController.x().onTrue(new RotateToAngleCMD(drivetrain, 120));
        driveController.y().onTrue(new RotateToAngleCMD(drivetrain, 180));
        driveController.povLeft().onTrue(new RotateToAngleCMD(drivetrain, 240));
        driveController.povRight().onTrue(new RotateToAngleCMD(drivetrain, 300));
        driveController.rightTrigger().onTrue(new RotateToAngleCMD(drivetrain, 144));
        driveController.leftTrigger().onTrue(new RotateToAngleCMD(drivetrain, 216));
        // driveController.rightTrigger().onTrue(new RotateRelativeAngleCMD(drivetrain, 60));
        // driveController.leftTrigger().onTrue(new RotateRelativeAngleCMD(drivetrain, -60));
        /*driveController.a().onTrue(new limelight());*/
     }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
