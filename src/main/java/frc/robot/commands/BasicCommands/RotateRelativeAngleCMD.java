package frc.robot.commands.BasicCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.StateMachine;


public class RotateRelativeAngleCMD extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController thetaController;
    private final double relativeAngle;

    public RotateRelativeAngleCMD(CommandSwerveDrivetrain drivetrain, double relativeDegrees) {
        this.drivetrain = drivetrain;
        this.relativeAngle = Math.toRadians(relativeDegrees); // Convert to radians
        this.thetaController = new PIDController(7.0, 0.0, 0.0); // Using the same PID gains from drivetrain

        // Enable continuous input to handle angle wrap-around (-π to π)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();
        thetaController.setSetpoint(currentAngle + relativeAngle); // Set target heading
        System.out.println("targetAngle set " + relativeAngle);
    }

    @Override
    public void execute() {
        double currentAngle = drivetrain.getState().Pose.getRotation().getRadians(); // Get current heading
        double speed = thetaController.calculate(currentAngle); // PID calculates rotational speed
        System.out.println("currentAngle " + currentAngle + "\tspeed: " + speed);

        // Apply the calculated rotation speed
        drivetrain.setControl(new SwerveRequest.SysIdSwerveRotation().withRotationalRate(speed));
    }

    @Override
    public boolean isFinished() {
        var positionError = thetaController.getPositionError();
        System.out.println("Position error: " + positionError);
        return Math.abs(positionError) < Math.toRadians(2.0); // Stop if within 2 degrees
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SysIdSwerveRotation().withRotationalRate(0)); // Stop rotation
        System.out.println("We did it! RotateToAngleCMD finished!");
    }
}