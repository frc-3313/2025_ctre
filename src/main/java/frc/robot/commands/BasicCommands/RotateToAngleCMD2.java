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
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}