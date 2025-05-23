// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BasicCommands.*;
import frc.robot.commands.BasicCommands.Algae.AlgaeHeightCMD;
import frc.robot.commands.BasicCommands.Algae.AlgaeIntake;
import frc.robot.commands.BasicCommands.Algae.AlgaeScore;
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
    private final CANdleSystem candle = new CANdleSystem();
    private final StateMachine stateMachine = new StateMachine(candle);
    //private final StateMachine stateMachine = new StateMachine();

    private final Coral coral = new Coral(stateMachine);
    private final Elevator elevator = new Elevator(stateMachine);
    private final Climber climber = new Climber(stateMachine);
    //private final Algae algae = new Algae(stateMachine);
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipulator = new CommandXboxController(1);
    private final SendableChooser<Command> autoChooser;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(stateMachine);

    public RobotContainer() {

        NamedCommands.registerCommand("SetLeft", new InstantCommand(() -> stateMachine.setScoreLeft(true)));
        NamedCommands.registerCommand("SetRight", new InstantCommand(() -> stateMachine.setScoreLeft(false)));
        NamedCommands.registerCommand("SetHeightL2", new InstantCommand(() -> stateMachine.setScoreHeight(1)));
        NamedCommands.registerCommand("SetHeightL3", new InstantCommand(() -> stateMachine.setScoreHeight(2))); 
        NamedCommands.registerCommand("SetHeightL4", new InstantCommand(() -> stateMachine.setScoreHeight(3)));
        NamedCommands.registerCommand("ScoreCoralHeight", new ScoreCoralHeightCMD(coral, elevator, stateMachine));
        //NamedCommands.registerCommand("GoToScoringPosition", new GoToScoringPosition(drivetrain, stateMachine));
        NamedCommands.registerCommand("DriveToAprilTag", new DriveToAprilTag(drivetrain, stateMachine));
        NamedCommands.registerCommand("CoralCMD", new CoralCMD(coral, stateMachine, 0.3));
        NamedCommands.registerCommand("ScoreCoralCMD", new ScoreCoralCMD(coral, elevator, stateMachine));
        NamedCommands.registerCommand("WaitForCoral", new WaitForCoral(stateMachine));
        // NamedCommands.registerCommand("SetPositionNegative45", new InstantCommand(() -> drivetrain.updateVisionOdometryAuto(-45)));
        // NamedCommands.registerCommand("SetPosition45", new InstantCommand(() -> drivetrain.updateVisionOdometryAuto(45)));
        // NamedCommands.registerCommand("SetPosition0", new InstantCommand(() -> drivetrain.updateVisionOdometryAuto(0)));
        // NamedCommands.registerCommand("SetPositionNegative135", new InstantCommand(() -> drivetrain.updateVisionOdometryAuto(-135)));
        // NamedCommands.registerCommand("SetPosition135", new InstantCommand(() -> drivetrain.updateVisionOdometryAuto(135)));
        // NamedCommands.registerCommand("SetPosition180", new InstantCommand(() -> drivetrain.updateVisionOdometryAuto(180)));
        // NamedCommands.registerCommand("WaitToStart", new WaitCommand(5));

                
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoMode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-drivetrain.getDriveY(driveController.getLeftY()) * stateMachine.getMaxSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-drivetrain.getDriveX(driveController.getLeftX())  * stateMachine.getMaxSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(drivetrain.getDriveRot(-driveController.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        // Mode switch bindings
        driveController.povLeft().onTrue(
            new InstantCommand(() -> stateMachine.SetDriveToSmart()));
        driveController.leftTrigger(.5).onTrue(
            new InstantCommand(() -> climber.ReleaseWings()));
        driveController.povRight().onTrue(
            new InstantCommand(() -> stateMachine.SetDriveToManual()));
        driveController.y().whileTrue(new ElevatorEject(elevator, stateMachine));

        driveController.start().onTrue(new ZeroGyro(drivetrain));
        // Shared bindings 

        //Climber
        //Grab - Left Bumper
        driveController.leftBumper().onTrue(
            new ClimbGrabPositionCMDPID(climber, stateMachine));
        //Climb - Right Bumper
        driveController.rightBumper().onTrue(
            new ClimbCMDPID(climber, stateMachine));

        manipulator.rightBumper().onTrue(
            new InstantCommand(() -> stateMachine.setScoreLeft(false)));
        manipulator.leftBumper().onTrue(
            new InstantCommand(() -> stateMachine.setScoreLeft(true)));

        manipulator.x().onTrue(new ReturnToNormal(coral, elevator, drivetrain));

        manipulator.povLeft().onTrue(
            new InstantCommand(() -> stateMachine.setScoreHeight(0)));
        manipulator.povDown().onTrue(
            new InstantCommand(() -> stateMachine.setScoreHeight(1)));
        manipulator.povRight().onTrue(
            new InstantCommand(() -> stateMachine.setScoreHeight(2)));
        manipulator.povUp().onTrue(
            new InstantCommand(() -> stateMachine.setScoreHeight(3)));
        //algae commands
        // manipulator.leftStick().onTrue(
        //     new InstantCommand(() -> stateMachine.setAlgaeHeight(0)));
        // manipulator.rightStick().onTrue(
        //     new InstantCommand(() -> stateMachine.setAlgaeHeight(1)));
        // //Button Right bumper - Intakes Algae
        // manipulator.rightBumper().whileTrue(
        //         new SequentialCommandGroup(
        //            // new AlgaeHeightCMD(elevator, stateMachine, true),
        //             new AlgaeIntake(algae, elevator, stateMachine)
        //         )    
        // );
        // manipulator.rightBumper().onFalse(new InstantCommand(() -> algae.StopIntake()));
        // manipulator.leftBumper().whileTrue(
        //     new SequentialCommandGroup(
        //        // new AlgaeHeightCMD(elevator, stateMachine, false),
        //         new AlgaeScore(algae, elevator, stateMachine)
        //     )    
        // );


        //manipulator
            //Button A - Intake
        manipulator.a().onTrue(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SmartIntake(stateMachine, coral, drivetrain, driveController), // Smart mode command
                    new CoralScoreDrive(stateMachine, drivetrain, driveController)),
                new CoralCMD(coral, stateMachine, 0.3),                           // Manual mode command
                stateMachine::IsDriveModeSmart                                    // Condition
            )
        );

        //Button B - Smart Drive
        manipulator.b().onTrue(
            new ConditionalCommand(
                new CoralScoreDrive(stateMachine, drivetrain, driveController),
                Commands.none(),
                stateMachine::IsDriveModeSmart
            )
        );

        //Button Right Trigger - Scores Right
        manipulator.rightTrigger().onTrue(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> stateMachine.setScoreLeft(false)),
                    //new GoToScoringPosition(drivetrain, stateMachine),
                    new ParallelCommandGroup
                    (
                        new DriveToAprilTag(drivetrain, stateMachine),
                        new ScoreCoralHeightCMD(coral, elevator, stateMachine)
                    ),
                    new ScoreCoralCMD(coral, elevator, stateMachine)
                ),                                                  // Smart mode (none for onTrue, handled onFalse)
                new ScoreCoralCMD(coral, elevator, stateMachine),                // Manual mode
                stateMachine::IsDriveModeSmart
            )
        );

        //Button Left Trigger - Scores Left
        manipulator.leftTrigger().onTrue(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> stateMachine.setScoreLeft(true)),
                    //new GoToScoringPosition(drivetrain, stateMachine),
                    new ParallelCommandGroup
                    (
                        new DriveToAprilTag(drivetrain, stateMachine),
                        new ScoreCoralHeightCMD(coral, elevator, stateMachine)
                    ),
                    new ScoreCoralCMD(coral, elevator, stateMachine)
                ),
                new ScoreCoralCMD(coral, elevator, stateMachine),
                stateMachine::IsDriveModeSmart
            )
        );

        //Button Y - In manual mode sends elevator up
        manipulator.y().onTrue(
            new ConditionalCommand(
                Commands.none(),
                new ScoreCoralHeightCMD(coral, elevator, stateMachine),
                stateMachine::IsDriveModeSmart
            )
        );
     }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    // public void setAlgaeStow()
    // {
    //     // An example command will be run in autonomous
    //     /* Run the path selected from the auto chooser */
    //     algae.setPos(Constants.Algae.TilterStorePos);
    // }
}
