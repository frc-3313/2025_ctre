package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.StateMachine;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@Logged
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final Field2d field2d = new Field2d();
    
    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private SlewRateLimiter x_speedLimiter = new SlewRateLimiter(6.0);
    private SlewRateLimiter y_speedLimiter = new SlewRateLimiter(6.0);
    private SlewRateLimiter rot_speedLimiter = new SlewRateLimiter(6.0);

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    //private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
    //private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        StateMachine stateMachine,
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        StateMachine stateMachine,
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        StateMachine stateMachine,
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

    }


    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        updateVisionOdometry();

        SmartDashboard.putNumber("swerve position", this.getState().Pose.getRotation().getDegrees());
        field2d.setRobotPose(getState().Pose);
        SmartDashboard.putData("field", field2d);
        
    }
    /*private void LimelightPoseTracker {
        LimelightResults results = Limelighthelpers.getLatestResults("limelight");
        double ta = LimelightHelpers.getTA("limelight");
        double minTargetArea = 5.0;

        if (ta >= minTargetArea) {
            double[] botPoseOrb = results.targetingResults.botpose_orb;
            double yaw = botPoseOrb[5];
        }
    }*/

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public void zeroGyro()
    {
        var swerveState = super.getState();
        var pose = swerveState.Pose;
        Pose2d newPose;
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        {   
           newPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(0));
        }
        else
        {
            newPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(Math.PI));

        }
        this.resetPose(newPose);
    }
    public void zeroGyroAuto(double startingAngle)
    {
        var swerveState = super.getState();
        var pose = swerveState.Pose;
        Pose2d newPose;
        newPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(startingAngle));

        // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        // {   
        //    newPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(Math.PI));
        // }
        // else
        // {
        //     newPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(Math.toRadians(-45)));

        // }
        this.resetPose(newPose);
    }

  public void updateVisionOdometry()
  {
    boolean doRejectUpdate = false;

    SmartDashboard.putBoolean("Limelight 2", LimelightHelpers.getTV(Constants.Limelight.LEFT));
    LimelightHelpers.SetRobotOrientation(Constants.Limelight.LEFT, getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight.LEFT);
    if (mt2 != null)
    {
        if(Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            boolean tv = LimelightHelpers.getTV(Constants.Limelight.LEFT);
            double tx = LimelightHelpers.getTX(Constants.Limelight.LEFT);
            int tid = (int)LimelightHelpers.getFiducialID(Constants.Limelight.LEFT); // Target ID (AprilTag)
            Pose2d pose = mt2.pose;
            // Check if a valid target is detected
            if (tv) 
            {

                // Get the target's known pose
                Optional<Pose3d> targetPose = fieldLayout.getTagPose(tid);
                if (targetPose.isPresent())
                {
                    pose = new Pose2d(pose.getX(), pose.getY(), targetPose.get().getRotation().toRotation2d().minus(Rotation2d.fromDegrees(tx)));
                }

            }
            setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
    }
  }
    


  private void configureAutoBuilder() {
  try {
    var config = RobotConfig.fromGUISettings();
    AutoBuilder.configure(
        () -> getState().Pose,   // Supplier of current robot pose
        this::resetPose,         // Consumer for seeding pose against auto
        () -> getState().Speeds, // Supplier of current robot speeds
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        (speeds, feedforwards) -> setControl(
            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
        ),
        new PPHolonomicDriveController(
            // PID constants for translation
            new PIDConstants(10, 0, 0),
            // PID constants for rotation
            new PIDConstants(7, 0, 0)
        ),
        config,
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this // Subsystem for requirements
        );
    } catch (Exception ex) {
        DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public double getDriveX(double input)
  {
    if (input > .1)
    {    
        double speed = (input - .1 )/.9;
        speed = speed * Math.abs(speed);
        return x_speedLimiter.calculate(speed);
    }
    else if (input < -.1)
    {    
        double speed = (input + .1 )/.9;
        speed = speed * Math.abs(speed);
        return x_speedLimiter.calculate(speed);
    }
    else
        return x_speedLimiter.calculate(0);
  }
  public double getDriveY(double input)
  {
    if (input > .1)
    {    
        double speed = (input - .1 )/.9;
        speed = speed * Math.abs(speed);
        return y_speedLimiter.calculate(speed);
    }
    else if (input < -.1)
    {    
        double speed = (input + .1 )/.9;
        speed = speed * Math.abs(speed);
        return y_speedLimiter.calculate(speed);
    }
    else
        return y_speedLimiter.calculate(0);

  }
  public double getDriveRot(double input)
  {
    if (input > .1)
    {    
        double speed = (input - .1 )/.9;
        speed = speed * Math.abs(speed);
        return rot_speedLimiter.calculate(speed);
    }
    else if (input < -.1)
    {    
        double speed = (input + .1 )/.9;
        speed = speed * Math.abs(speed);
        return rot_speedLimiter.calculate(speed);
    }
    else
        return rot_speedLimiter.calculate(0);

  }
  public Command updateVisionCommand(double startingAngle) { return this.runOnce(() -> this.updateVisionOdometryAuto(startingAngle)); }
  public void updateVisionOdometryAuto(double startingAngle)
  {
    zeroGyroAuto(startingAngle);
    boolean doRejectUpdate = false;
    SmartDashboard.putBoolean("Limelight 2", LimelightHelpers.getTV(Constants.Limelight.LEFT));
    LimelightHelpers.SetRobotOrientation(Constants.Limelight.LEFT, getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight.LEFT);
    if(mt2 != null)
    {
        if(Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            boolean tv = LimelightHelpers.getTV(Constants.Limelight.LEFT);
            double tx = LimelightHelpers.getTX(Constants.Limelight.LEFT);
            int tid = (int)LimelightHelpers.getFiducialID(Constants.Limelight.LEFT); // Target ID (AprilTag)
            Pose2d pose = mt2.pose;
    // Check if a valid target is detected
            if (tv) 
            {

                // Get the target's known pose
                Optional<Pose3d> targetPose = fieldLayout.getTagPose(tid);
                if(targetPose.isPresent())
                pose = new Pose2d(pose.getX(), pose.getY(), targetPose.get().getRotation().toRotation2d().minus(Rotation2d.fromDegrees(tx)));
            
            }
            setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }

    }
  }
}
  
