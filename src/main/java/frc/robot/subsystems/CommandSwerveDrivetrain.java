package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance: 0 degrees (forward is toward red wall) */
    /* Red alliance: 180 degrees (forward is toward blue wall - flipped perspective) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    /* Track last alliance to detect changes */
    private Alliance m_lastAlliance = null;

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
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
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
         * IMPORTANT FIX: Always update if alliance changes, not just on first apply or disabled.
         * This fixes the bug where controls are reversed on first connection.
         */
        DriverStation.getAlliance().ifPresent(allianceColor -> {
            // Check if alliance changed or never applied before
            boolean allianceChanged = (m_lastAlliance != null && m_lastAlliance != allianceColor);
            boolean shouldApply = !m_hasAppliedOperatorPerspective || DriverStation.isDisabled() || allianceChanged;

            if (shouldApply) {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
                m_lastAlliance = allianceColor;

                // Debug log when perspective changes
                System.out.println("🔄 Operator Perspective Updated: " +
                    (allianceColor == Alliance.Blue ? "BLUE (0°)" : "RED (180°)"));
            }
        });

        // Update pose estimation with MegaTag2 vision measurements
        updateVisionMeasurements();
        SmartDashboard.putBoolean("Vision/Enabled", true);

        // Debug: Log gyro yaw to SmartDashboard for troubleshooting
        SmartDashboard.putNumber("Drivetrain/Gyro Yaw (degrees)", getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Drivetrain/Pose X (m)", getState().Pose.getX());
        SmartDashboard.putNumber("Drivetrain/Pose Y (m)", getState().Pose.getY());
    }

    /**
     * Updates the pose estimator with vision measurements from Limelight MegaTag2.
     *
     * <p>Uses Mechanical Advantage Team 6328's aggressive vision fusion strategy:
     * <ul>
     *   <li>Accepts all measurements with tagCount ≥ 1 (no hard rejection limits)</li>
     *   <li>Adaptive standard deviation based on distance and tag count</li>
     *   <li>Very low std dev (0.05-0.15m) for maximum trust in vision</li>
     *   <li>MegaTag2 provides inherent multi-tag validation</li>
     * </ul>
     *
     * <p>This fuses vision data with wheel odometry using WPILib's Kalman filter.
     * Vision measurements retroactively correct odometry drift with timestamp-based fusion.
     */
    private void updateVisionMeasurements() {
        // Tell Limelight the robot's current orientation for better pose estimation
        LimelightHelpers.SetRobotOrientation("limelight", getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // Get MegaTag2 pose estimate from Limelight based on alliance
        // MegaTag2 uses multiple tags for more accurate pose estimation
        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Red);
        SmartDashboard.putString("Vision/CurrentAlliance", currentAlliance.toString());

        // Always use wpiBlue coordinate system for pose estimation
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        SmartDashboard.putString("Vision/UsingPoseMethod", "wpiBlue_MegaTag2");
        SmartDashboard.putString("Vision/CoordinateSystem", "Always wpiBlue (Red alliance uses blue coords)");

        // Telemetry: Vision data availability and quality metrics
        SmartDashboard.putBoolean("Vision/HasValidData", poseEstimate != null && poseEstimate.tagCount > 0);
        if (poseEstimate != null) {
            SmartDashboard.putNumber("Vision/TagCount", poseEstimate.tagCount);
            SmartDashboard.putNumber("Vision/AvgTagDist (m)", poseEstimate.avgTagDist);
            SmartDashboard.putNumber("Vision/TagSpan (m)", poseEstimate.tagSpan);
            SmartDashboard.putNumber("Vision/Latency (ms)", poseEstimate.latency);
            SmartDashboard.putNumber("Vision/PoseX", poseEstimate.pose.getX());
            SmartDashboard.putNumber("Vision/PoseY", poseEstimate.pose.getY());
            SmartDashboard.putNumber("Vision/PoseRotation", poseEstimate.pose.getRotation().getDegrees());
        }

        // Only use vision data if we have valid tag data
        if (poseEstimate != null && poseEstimate.tagCount > 0) {
            // Get current odometry pose for comparison
            Pose2d currentPose = getState().Pose;

            // Calculate distance between vision pose and current odometry pose
            double poseDifference = currentPose.getTranslation().getDistance(poseEstimate.pose.getTranslation());
            double rotationDifference = Math.abs(
                currentPose.getRotation().minus(poseEstimate.pose.getRotation()).getDegrees()
            );

            // Debug: Show pose difference
            SmartDashboard.putNumber("Vision/PoseDifference (m)", poseDifference);
            SmartDashboard.putNumber("Vision/RotationDifference (deg)", rotationDifference);

            // ========== MECHANICAL ADVANTAGE 6328 VISION VALIDATION ALGORITHM ==========
            // Strategy: Aggressive vision trust with adaptive standard deviation
            // No hard rejection limits - MegaTag2 provides inherent validation

            // Calculate vision measurement standard deviation based on:
            // 1. Distance to tags (farther = less accurate)
            // 2. Number of tags (more tags = more accurate)
            // Lower std dev = MORE trust in vision (stronger correction)
            // Higher std dev = LESS trust in vision (weaker correction)

            // Base standard deviations - AGGRESSIVE trust in vision
            double xyStdDev = 0.1;      // XY position: Very low = MAXIMUM trust
            double thetaStdDev = 2.0;   // Rotation: Low = High trust

            double avgTagDistance = poseEstimate.avgTagDist;

            // Distance-based adjustment: Reduce trust at far distances (>4m)
            if (avgTagDistance > 4.0) {
                xyStdDev *= 1.5;        // Slightly reduce trust
                thetaStdDev *= 1.5;
            }

            // Multiple tag bonus: MAXIMUM trust with 2+ tags
            if (poseEstimate.tagCount >= 2) {
                xyStdDev *= 0.5;        // Double the trust (half the std dev)
                thetaStdDev *= 0.5;
            }

            // Final std devs used:
            // - Single tag, close (<4m): XY=0.1m, Theta=2.0°
            // - Single tag, far (>4m):   XY=0.15m, Theta=3.0°
            // - Multi tag, close:        XY=0.05m, Theta=1.0° (MAXIMUM TRUST)
            // - Multi tag, far:          XY=0.075m, Theta=1.5°

            SmartDashboard.putNumber("Vision/XYStdDev", xyStdDev);
            SmartDashboard.putNumber("Vision/ThetaStdDev", thetaStdDev);
            SmartDashboard.putBoolean("Vision/MeasurementRejected", false); // No rejection in MA strategy

            // Create standard deviation matrix [x, y, theta]
            var visionStdDevs = new Matrix<>(N3.instance, N1.instance);
            visionStdDevs.set(0, 0, xyStdDev);
            visionStdDevs.set(1, 0, xyStdDev);
            visionStdDevs.set(2, 0, Math.toRadians(thetaStdDev));

            // Add vision measurement to pose estimator with timestamp and std devs
            addVisionMeasurement(
                poseEstimate.pose,
                poseEstimate.timestampSeconds,
                visionStdDevs
            );

            SmartDashboard.putBoolean("Vision/MeasurementAccepted", true);
        } else {
            SmartDashboard.putBoolean("Vision/MeasurementAccepted", false);
        }
    }

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

    /**
     * Configures PathPlanner AutoBuilder for autonomous path following.
     * This sets up the holonomic drive controller and robot configuration.
     */
    private void configurePathPlanner() {
        try {
            // Create RobotConfig manually for swerve drive
            // Try to load from GUI settings first, fallback to manual config if it fails
            RobotConfig config;
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                // Fallback: Create manual config for simulation/testing
                // These values should be tuned for your specific robot
                DriverStation.reportWarning("Using fallback PathPlanner config - create pathplanner/settings.json using PathPlanner GUI", false);

                // Robot mass in kg (estimate: ~60 lbs = ~27 kg)
                double massKg = 27.0;

                // Moment of inertia in kg*m^2 (estimate for square robot)
                double MOI = 6.0;

                // Module locations from TunerConstants (convert inches to meters)
                double modulePosMeters = 0.345; // 13.5825 inches = 0.345 meters

                // Create ModuleConfig with Kraken X60 motor
                com.pathplanner.lib.config.ModuleConfig moduleConfig =
                    new com.pathplanner.lib.config.ModuleConfig(
                        0.1016, // wheel radius in meters (4 inches)
                        10.59, // max drive speed m/s from TunerConstants
                        1.2, // wheel COF (coefficient of friction)
                        DCMotor.getKrakenX60(1), // Kraken X60 drive motor
                        6.03, // drive gear ratio from TunerConstants
                        40.0, // drive motor current limit (amps)
                        1 // number of drive motors per module
                    );

                config = new RobotConfig(
                    massKg,
                    MOI,
                    moduleConfig,
                    new Translation2d(modulePosMeters, modulePosMeters), // Front Left
                    new Translation2d(modulePosMeters, -modulePosMeters), // Front Right
                    new Translation2d(-modulePosMeters, modulePosMeters), // Back Left
                    new Translation2d(-modulePosMeters, -modulePosMeters) // Back Right
                );
            }

            // Configure AutoBuilder for a holonomic drivetrain
            AutoBuilder.configure(
                () -> getState().Pose,  // Supplier of current robot pose
                this::resetPoseForPath, // Consumer to reset robot pose
                this::getCurrentRobotChassisSpeeds, // Supplier of current robot-relative chassis speeds
                (speeds, feedforwards) -> {
                    setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
                    // Debug output
                    System.out.println("PathPlanner commanding: vX=" + speeds.vxMetersPerSecond +
                                     " vY=" + speeds.vyMetersPerSecond +
                                     " omega=" + speeds.omegaRadiansPerSecond);
                }, // Consumer of ChassisSpeeds to drive the robot
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants (increased for better tracking)
                    new PIDConstants(9.0, 1.2, 0.15), // Rotation PID constants (slightly higher P and I to complete full rotation)
                    0.02 // Control loop period in seconds (20ms)
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner AutoBuilder: " + e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * Returns the current robot-relative chassis speeds.
     *
     * @return Current robot-relative chassis speeds
     */
    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getState().Speeds;
    }

    /**
     * Resets the robot's pose to the given pose. This is used by PathPlanner.
     *
     * @param pose The new pose for the robot
     */
    private void resetPoseForPath(Pose2d pose) {
        try {
            resetPose(pose);
        } catch (Exception e) {
            DriverStation.reportError("Failed to reset pose for path", e.getStackTrace());
        }
    }
}
