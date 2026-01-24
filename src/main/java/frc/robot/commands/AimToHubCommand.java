// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Command to aim robot rotation at nearest Hub face using pose estimation.
 *
 * <p>The robot uses its odometry (fused with vision measurements from Limelight)
 * to determine its field position, then calculates the angle to the nearest
 * Hub face belonging to its alliance. Uses proportional control to automatically
 * aim at the Hub while the driver retains full translational control (forward/strafe).
 *
 * <p>Alliance-aware: Blue alliance aims at Blue Hub (tags 17-28),
 *                 Red alliance aims at Red Hub (tags 1-12)
 */
public class AimToHubCommand extends Command {

    // ========== Control Constants ==========

    // Proportional control constant for pose-based aiming
    private static final double KP_POSE = 0.06;

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
     * Represents a Hub face position with its pose and associated AprilTag IDs for each alliance.
     */
    public static class HubFace {
        Pose2d pose;
        int blueID;
        int redID;

        HubFace(Pose2d pose, int blueID, int redID) {
            this.blueID = blueID;
            this.pose = pose;
            this.redID = redID;
        }

        int getID() {
            return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue
                    ? this.blueID
                    : this.redID;
        }
    }

    /**
     * FRC 2026 REBUILT - Hub merkez koordinatları
     *
     * CSV'den hesaplanan (2026-rebuilt-andymark.csv)
     * Saha boyutu: 16.518m x 8.043m
     *
     * Hub merkez noktaları (12 AprilTag'in ortalaması):
     * - Blue Hub Center: X: 4.552m, Y: 4.021m (Tag'ler: 17-28)
     * - Red Hub Center: X: 11.961m, Y: 4.021m (Tag'ler: 1-12)
     */
    private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.552, 4.021);
    private static final Translation2d RED_HUB_CENTER = new Translation2d(11.961, 4.021);

    public static List<HubFace> hubFaces = Arrays.asList(
            // BLUE HUB (Tag 17-28) - Sol taraf, resmi açılarla
            // Tag 17: 0° (alt yüz)
            new HubFace(new Pose2d(4.649, 0.631, new Rotation2d(Units.degreesToRadians(0))), 17, 0),
            // Tag 18: -90° (270°)
            new HubFace(new Pose2d(4.612, 3.418, new Rotation2d(Units.degreesToRadians(-90))), 18, 0),
            // Tag 19: 0°
            new HubFace(new Pose2d(5.215, 3.666, new Rotation2d(Units.degreesToRadians(0))), 19, 0),
            // Tag 20: 0°
            new HubFace(new Pose2d(5.215, 4.021, new Rotation2d(Units.degreesToRadians(0))), 20, 0),
            // Tag 21: 90°
            new HubFace(new Pose2d(4.612, 4.625, new Rotation2d(Units.degreesToRadians(90))), 21, 0),
            // Tag 22: 0° (alt yüz)
            new HubFace(new Pose2d(4.649, 7.411, new Rotation2d(Units.degreesToRadians(0))), 22, 0),
            // Tag 23: 180° (üst yüz)
            new HubFace(new Pose2d(4.574, 7.411, new Rotation2d(Units.degreesToRadians(180))), 23, 0),
            // Tag 24: 90°
            new HubFace(new Pose2d(4.256, 4.625, new Rotation2d(Units.degreesToRadians(90))), 24, 0),
            // Tag 25: 180°
            new HubFace(new Pose2d(4.008, 4.377, new Rotation2d(Units.degreesToRadians(180))), 25, 0),
            // Tag 26: 180°
            new HubFace(new Pose2d(4.008, 4.021, new Rotation2d(Units.degreesToRadians(180))), 26, 0),
            // Tag 27: -90° (270°)
            new HubFace(new Pose2d(4.256, 3.418, new Rotation2d(Units.degreesToRadians(-90))), 27, 0),
            // Tag 28: 180° (alt yüz)
            new HubFace(new Pose2d(4.574, 0.631, new Rotation2d(Units.degreesToRadians(180))), 28, 0),

            // RED HUB (Tag 1-12) - Sağ taraf, resmi açılarla
            // Tag 1: 180° (üst yüz)
            new HubFace(new Pose2d(11.864, 7.411, new Rotation2d(Units.degreesToRadians(180))), 0, 1),
            // Tag 2: 90°
            new HubFace(new Pose2d(11.901, 4.625, new Rotation2d(Units.degreesToRadians(90))), 0, 2),
            // Tag 3: 180°
            new HubFace(new Pose2d(11.298, 4.377, new Rotation2d(Units.degreesToRadians(180))), 0, 3),
            // Tag 4: 180°
            new HubFace(new Pose2d(11.298, 4.021, new Rotation2d(Units.degreesToRadians(180))), 0, 4),
            // Tag 5: -90° (270°)
            new HubFace(new Pose2d(11.901, 3.418, new Rotation2d(Units.degreesToRadians(-90))), 0, 5),
            // Tag 6: 180° (alt yüz)
            new HubFace(new Pose2d(11.864, 0.631, new Rotation2d(Units.degreesToRadians(180))), 0, 6),
            // Tag 7: 0° (üst yüz)
            new HubFace(new Pose2d(11.939, 7.411, new Rotation2d(Units.degreesToRadians(0))), 0, 7),
            // Tag 8: -90° (270°)
            new HubFace(new Pose2d(12.257, 3.418, new Rotation2d(Units.degreesToRadians(-90))), 0, 8),
            // Tag 9: 0°
            new HubFace(new Pose2d(12.505, 3.666, new Rotation2d(Units.degreesToRadians(0))), 0, 9),
            // Tag 10: 0°
            new HubFace(new Pose2d(12.505, 4.021, new Rotation2d(Units.degreesToRadians(0))), 0, 10),
            // Tag 11: 90°
            new HubFace(new Pose2d(12.257, 4.625, new Rotation2d(Units.degreesToRadians(90))), 0, 11),
            // Tag 12: 0° (alt yüz)
            new HubFace(new Pose2d(11.939, 0.631, new Rotation2d(Units.degreesToRadians(0))), 0, 12)
    );

    /**
     * FRC 2026 REBUILT - Saha Kenarı AprilTag ID'leri ve Pozisyonları
     *
     * Resmi WPILib koordinatları (2026-rebuilt-andymark.json)
     *
     * Saha kenarlarındaki Tag'ler (oyun elemanı marker'ları):
     * - Kırmızı taraf: Tag 13-16 (X: 16.499m, sağ duvar) - Z: 0.552m
     * - Mavi taraf: Tag 29-32 (X: 0.014m, sol duvar) - Z: 0.552m
     *
     * Not: Gerçek saha elemanı isimleri için oyun manualine bakın.
     */

    // Red Alliance - Saha kenarı Tag'leri (sağ duvar, X: 16.499m)
    public static final int RED_FIELD_TAG_13 = 13;  // Y: 7.392, Z: 0.552
    public static final int RED_FIELD_TAG_14 = 14;  // Y: 6.960, Z: 0.552
    public static final int RED_FIELD_TAG_15 = 15;  // Y: 4.312, Z: 0.552
    public static final int RED_FIELD_TAG_16 = 16;  // Y: 3.881, Z: 0.552

    // Blue Alliance - Saha kenarı Tag'leri (sol duvar, X: 0.014m)
    public static final int BLUE_FIELD_TAG_29 = 29; // Y: 0.651, Z: 0.552
    public static final int BLUE_FIELD_TAG_30 = 30; // Y: 1.083, Z: 0.552
    public static final int BLUE_FIELD_TAG_31 = 31; // Y: 3.730, Z: 0.552
    public static final int BLUE_FIELD_TAG_32 = 32; // Y: 4.162, Z: 0.552

    /**
     * Gets the desired approach rotation for a specific AprilTag/Hub face.
     *
     * @param id The AprilTag ID
     * @return The rotation the robot should face when approaching this Hub face
     */
    public static Rotation2d getHubRotation(double id) {
        for (HubFace hubFace : hubFaces) {
            if (hubFace.redID == id || hubFace.blueID == id) {
                return hubFace.pose.getRotation();
            }
        }
        return new Rotation2d();
    }

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

        // Debug: Show hub center being used
        SmartDashboard.putNumber("AimToHub/HubCenterX", hubCenter.getX());
        SmartDashboard.putNumber("AimToHub/HubCenterY", hubCenter.getY());
        SmartDashboard.putString("AimToHub/Alliance", currentAlliance.toString());

        // Calculate angle from robot to hub center
        Translation2d robotToHub = hubCenter.minus(robotPose.getTranslation());
        double distanceToHub = robotToHub.getNorm();
        Rotation2d angleToHub = new Rotation2d(robotToHub.getX(), robotToHub.getY());

        // Calculate rotation error (how much robot needs to turn)
        double rotationError = getModuloRotation(
            angleToHub.getDegrees() - robotPose.getRotation().getDegrees()
        );

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

        // Debug output
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
}
