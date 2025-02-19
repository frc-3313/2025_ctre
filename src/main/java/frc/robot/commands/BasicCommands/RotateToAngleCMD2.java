package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotateToAngleCMD2 extends Command {
    private final CommandSwerveDrivetrain swerveDrive;
    private final double targetAngle; // Target angle in degrees

    public RotateToAngleCMD2(CommandSwerveDrivetrain swerveDrive, double targetAngle) {
        this.swerveDrive = swerveDrive;
        this.targetAngle = targetAngle;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        System.out.println("Rotating to " + targetAngle + " degrees.");
    }

    @Override
    public void execute() {
        // Use Phoenix 6's built-in control for heading
        SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                .withVelocityX(0.0) // No translational movement
                .withVelocityY(0.0)
                .withTargetHeading(Rotation2d.fromDegrees(targetAngle)); // Rotate to target

        swerveDrive.setControl(driveRequest);
    }

    @Override
    public boolean isFinished() {
        // Check if we've reached the target angle
        double currentAngle = swerveDrive.getHeading().getDegrees();
        return Math.abs(targetAngle - currentAngle) < 2.0; // 2-degree threshold
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Rotation complete.");
        swerveDrive.setControl(new SwerveRequest.FieldCentric()); // Stop motion
    }
}