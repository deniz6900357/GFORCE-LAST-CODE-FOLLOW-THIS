// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

/**
 * Command to aim robot rotation at Hub center using simple geometric aiming.
 *
 * <p>The robot uses its odometry (fused with vision measurements from Limelight)
 * to determine its field position, then calculates the angle to the Hub center.
 * Uses proportional control to automatically aim at the Hub while the driver
 * retains full translational control (forward/strafe).
 *
 * <p>Alliance-aware: Blue alliance aims at Blue Hub (X: 4.552, Y: 4.021),
 *                 Red alliance aims at Red Hub (X: 11.988, Y: 4.021)
 */
public class AimToHubCommand extends Command {

    // ========== Control Constants ==========

    // Proportional control constant for pose-based aiming
    private static final double KP_POSE = 0.08;

    // Minimum command to overcome friction - prevents robot from getting "stuck"
    // when very close to target
    private static final double MIN_COMMAND = 0.02;

    // Threshold in degrees - don't apply min_command if error is smaller than this
    private static final double ERROR_THRESHOLD = 1.0;

    // Tolerance in degrees - command finishes when within this tolerance for autonomous
    private static final double FINISH_TOLERANCE_DEGREES = 3.0;

    // Number of consecutive loops robot must be within tolerance before finishing
    private static final int TOLERANCE_LOOP_COUNT = 5;

    // Joystick deadband - same as normal drive (10%)
    private static final double DEADBAND = 0.1;

    // ========== Subsystems and Inputs ==========

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier leftYSupplier;
    private final DoubleSupplier leftXSupplier;
    private final double maxSpeed;
    private final double maxAngularRate;
    private final boolean finishWhenAtTarget;

    // ========== State Tracking ==========

    // Counter to track how many loops we've been within tolerance
    private int withinToleranceCount = 0;

    /**
     * FRC 2026 REBUILT - Hub merkez koordinatları
     *
     * MUST MATCH FieldConstants.Hub.topCenterPoint for consistency with LaunchCalculator!
     *
     * Hub merkez noktaları (12 AprilTag'in geometrik merkezi):
     * - Blue Hub Center: X: 4.552m, Y: 4.021m (Tag'ler: 17-28)
     * - Red Hub Center: X: 11.988m, Y: 4.021m (Tag'ler: 1-12)
     */
    private static final Translation2d BLUE_HUB_CENTER = new Translation2d(
        frc.robot.FieldConstants.Hub.topCenterPoint.getX(),
        frc.robot.FieldConstants.Hub.topCenterPoint.getY()
    );
    private static final Translation2d RED_HUB_CENTER = new Translation2d(
        frc.robot.FieldConstants.fieldLength - frc.robot.FieldConstants.Hub.topCenterPoint.getX(),
        frc.robot.FieldConstants.Hub.topCenterPoint.getY()
    );

    /**
     * Normalizes an angle to the range [-180, 180] degrees.
     *
     * @param rawYaw The raw yaw angle in degrees
     * @return Normalized angle in degrees
     */
    public static double getModuloRotation(double rawYaw) {
        double modified = (Math.abs(rawYaw) % (360)) * Math.signum(rawYaw);
        if (modified < -180) modified += 360;
        if (modified > 180) modified -= 360;
        return modified;
    }

    /**
     * Creates a new AimToHubCommand for teleop (runs until interrupted).
     *
     * @param drivetrain The swerve drivetrain subsystem
     * @param leftYSupplier Supplier for left joystick Y (forward/backward)
     * @param leftXSupplier Supplier for left joystick X (left/right strafe)
     * @param maxSpeed Maximum translational speed
     * @param maxAngularRate Maximum rotational speed
     */
    public AimToHubCommand(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier leftYSupplier,
        DoubleSupplier leftXSupplier,
        double maxSpeed,
        double maxAngularRate
    ) {
        this(drivetrain, leftYSupplier, leftXSupplier, maxSpeed, maxAngularRate, false);
    }

    /**
     * Creates a new AimToHubCommand.
     *
     * @param drivetrain The swerve drivetrain subsystem
     * @param leftYSupplier Supplier for left joystick Y (forward/backward)
     * @param leftXSupplier Supplier for left joystick X (left/right strafe)
     * @param maxSpeed Maximum translational speed
     * @param maxAngularRate Maximum rotational speed
     * @param finishWhenAtTarget If true, command ends when aligned (for autonomous). If false, runs until interrupted (for teleop)
     */
    public AimToHubCommand(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier leftYSupplier,
        DoubleSupplier leftXSupplier,
        double maxSpeed,
        double maxAngularRate,
        boolean finishWhenAtTarget
    ) {
        this.drivetrain = drivetrain;
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.finishWhenAtTarget = finishWhenAtTarget;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        withinToleranceCount = 0;
        SmartDashboard.putBoolean("AimToHub/Running", true);
        System.out.println("AimToHubCommand started - Robot will aim at Hub!");
    }

    @Override
    public void execute() {
        // Get joystick inputs for driver control with deadband
        double leftY = leftYSupplier.getAsDouble();
        double leftX = leftXSupplier.getAsDouble();

        // Apply deadband (10%)
        if (Math.abs(leftY) < DEADBAND) leftY = 0.0;
        if (Math.abs(leftX) < DEADBAND) leftX = 0.0;

        double forwardSpeed = -leftY * maxSpeed;  // Negative for correct direction
        double strafeSpeed = -leftX * maxSpeed;   // Negative for correct direction

        // Get robot pose from drivetrain odometry (fused with vision)
        Pose2d robotPose = drivetrain.getState().Pose;

        // Get current alliance and select hub center
        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Translation2d hubCenter = (currentAlliance == Alliance.Blue)
            ? BLUE_HUB_CENTER
            : RED_HUB_CENTER;

        // Robot merkezinden hub'a olan açıyı hesapla
        Translation2d robotToHub = hubCenter.minus(robotPose.getTranslation());
        double distanceToHub = robotToHub.getNorm();
        Rotation2d angleToHub = new Rotation2d(robotToHub.getX(), robotToHub.getY());

        // Calculate rotation error (how much robot needs to turn)
        double rotationError = getModuloRotation(
            angleToHub.getDegrees() - robotPose.getRotation().getDegrees()
        );

        // Debug: Show aiming info
        SmartDashboard.putNumber("AimToHub/HubCenterX", hubCenter.getX());
        SmartDashboard.putNumber("AimToHub/HubCenterY", hubCenter.getY());
        SmartDashboard.putString("AimToHub/Alliance", currentAlliance.toString());

        double steeringAdjust = 0.0;

        // Apply proportional control
        if (Math.abs(rotationError) > ERROR_THRESHOLD) {
            steeringAdjust = KP_POSE * rotationError;

            // Add minimum command to overcome friction
            if (rotationError < 0) {
                steeringAdjust -= MIN_COMMAND;
            } else {
                steeringAdjust += MIN_COMMAND;
            }
        } else {
            steeringAdjust = KP_POSE * rotationError;
        }

        // Clamp steering to max angular rate
        steeringAdjust = Math.max(-maxAngularRate, Math.min(maxAngularRate, steeringAdjust));

        // Track tolerance for autonomous finishing
        if (Math.abs(rotationError) <= FINISH_TOLERANCE_DEGREES) {
            withinToleranceCount++;
        } else {
            withinToleranceCount = 0;
        }

        // Extended debug output
        SmartDashboard.putNumber("AimToHub/RotationError", rotationError);
        SmartDashboard.putNumber("AimToHub/TargetAngle", angleToHub.getDegrees());
        SmartDashboard.putNumber("AimToHub/RobotRotation", robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("AimToHub/SteeringAdjust", steeringAdjust);
        SmartDashboard.putNumber("AimToHub/DistanceToHub", distanceToHub);
        SmartDashboard.putString("AimToHub/TargetHub", currentAlliance == Alliance.Blue ? "Blue Hub" : "Red Hub");
        SmartDashboard.putNumber("AimToHub/ToleranceCount", withinToleranceCount);
        SmartDashboard.putNumber("AimToHub/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("AimToHub/StrafeSpeed", strafeSpeed);

        // Apply field-centric drive with automatic rotation adjustment
        drivetrain.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(forwardSpeed)
                .withVelocityY(strafeSpeed)
                .withRotationalRate(steeringAdjust)
                .withDriveRequestType(DriveRequestType.Velocity)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
        SmartDashboard.putBoolean("AimToHub/Running", false);
        System.out.println("AimToHubCommand ended! Interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        // If configured to finish when at target (autonomous mode)
        if (finishWhenAtTarget) {
            return withinToleranceCount >= TOLERANCE_LOOP_COUNT;
        }
        // Otherwise run until interrupted (teleop mode)
        return false;
    }

    /**
     * Gets the current robot pose.
     * Used by shooter subsystem for distance calculations.
     *
     * @return Current robot pose
     */
    public Pose2d getLastPredictedPose() {
        return drivetrain.getState().Pose;
    }
}
