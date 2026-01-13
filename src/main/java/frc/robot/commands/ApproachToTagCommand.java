// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Arrays;
import java.util.List;

/**
 * Command to aim robot at AprilTag using Limelight vision tracking.
 *
 * Based on Limelight visual servoing tutorial:
 * https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-with-visual-servoing
 *
 * The robot uses the tx (horizontal offset) value from Limelight
 * with proportional control to automatically aim at the AprilTag
 * while the driver retains full translational control.
 */
public class ApproachToTagCommand extends ConditionalCommand {

    // Proportional control constant - adjust based on testing
    // Negative because positive tx means target is to the right,
    // so we need to rotate counter-clockwise (negative rotation)
    private static final double KP = -0.05;

    // Minimum command to overcome friction - prevents robot from getting "stuck"
    // when very close to target
    private static final double MIN_COMMAND = 0.02;

    // Threshold in degrees - don't apply min_command if error is smaller than this
    private static final double ERROR_THRESHOLD = 1.0;

    // Joystick deadband - same as normal drive (10%)
    private static final double DEADBAND = 0.1;

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
     * FRC 2026 REBUILT - Hub AprilTag pozisyonları ve ID'leri
     *
     * Saha düzeni (Scoring Table'dan bakınca):
     * - Kırmızı Hub: Sol tarafta (Tag'ler: 1-12 arası)
     * - Mavi Hub: Sağ tarafta (Tag'ler: 17-28 arası)
     *
     * Hub altıgen şeklinde, her yüzde 2 tag var (üst/alt veya yan yana)
     * Açılar robotun Hub'a yaklaşırken bakması gereken yönü belirtir.
     */
    public static List<HubFace> hubFaces = Arrays.asList(
            // Hub Face - Üst (Scoring Table'a uzak, yukarı bakan)
            // Kırmızı: 7, 6 | Mavi: 17, 28
            new HubFace(new Pose2d(4.50, 6.00, new Rotation2d(Units.degreesToRadians(180))), 17, 7),
            new HubFace(new Pose2d(4.50, 6.00, new Rotation2d(Units.degreesToRadians(180))), 28, 6),

            // Hub Face - Sağ Üst
            // Kırmızı: 8/5 (üst/alt), 4/3 | Mavi: 18/27, 26/25
            new HubFace(new Pose2d(5.50, 5.00, new Rotation2d(Units.degreesToRadians(120))), 18, 8),
            new HubFace(new Pose2d(5.50, 5.00, new Rotation2d(Units.degreesToRadians(120))), 27, 5),
            new HubFace(new Pose2d(5.50, 5.00, new Rotation2d(Units.degreesToRadians(60))), 26, 4),
            new HubFace(new Pose2d(5.50, 5.00, new Rotation2d(Units.degreesToRadians(60))), 25, 3),

            // Hub Face - Alt (Scoring Table'a yakın, aşağı bakan)
            // Kırmızı: 12, 1 | Mavi: 22, 23
            new HubFace(new Pose2d(4.50, 2.00, new Rotation2d(Units.degreesToRadians(0))), 22, 12),
            new HubFace(new Pose2d(4.50, 2.00, new Rotation2d(Units.degreesToRadians(0))), 23, 1),

            // Hub Face - Sol Üst
            // Kırmızı: 9/10, 11/2 | Mavi: 19/20, 21/24
            new HubFace(new Pose2d(3.50, 5.00, new Rotation2d(Units.degreesToRadians(-120))), 19, 9),
            new HubFace(new Pose2d(3.50, 5.00, new Rotation2d(Units.degreesToRadians(-120))), 20, 10),
            new HubFace(new Pose2d(3.50, 5.00, new Rotation2d(Units.degreesToRadians(-60))), 21, 11),
            new HubFace(new Pose2d(3.50, 5.00, new Rotation2d(Units.degreesToRadians(-60))), 24, 2)
    );

    /**
     * FRC 2026 REBUILT - Diğer Saha Elemanları AprilTag ID'leri
     *
     * CORAL STATION (Parça İstasyonları):
     * - Kırmızı: Tag 14 (üst), Tag 13 (alt) - Sol alt köşe
     * - Mavi: Tag 29 (üst), Tag 30 (alt) - Sağ üst köşe
     *
     * PROCESSOR (İşlemci):
     * - Kırmızı: Tag 16 (üst), Tag 15 (alt) - Sol duvar
     * - Mavi: Tag 31 (üst), Tag 32 (alt) - Sağ duvar
     */

    // Coral Station Tag ID'leri
    public static final int RED_CORAL_STATION_TOP = 14;
    public static final int RED_CORAL_STATION_BOTTOM = 13;
    public static final int BLUE_CORAL_STATION_TOP = 29;
    public static final int BLUE_CORAL_STATION_BOTTOM = 30;

    // Processor Tag ID'leri
    public static final int RED_PROCESSOR_TOP = 16;
    public static final int RED_PROCESSOR_BOTTOM = 15;
    public static final int BLUE_PROCESSOR_TOP = 31;
    public static final int BLUE_PROCESSOR_BOTTOM = 32;

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
     * Creates a new ApproachToTagCommand.
     *
     * @param drivetrain The swerve drivetrain subsystem
     * @param joystickSupplier Supplier for joystick X axis (for rotation adjustment)
     * @param leftYSupplier Supplier for left joystick Y (forward/backward)
     * @param leftXSupplier Supplier for left joystick X (left/right strafe)
     * @param maxSpeed Maximum translational speed
     * @param maxAngularRate Maximum rotational speed
     */
    public ApproachToTagCommand(
        CommandSwerveDrivetrain drivetrain,
        java.util.function.DoubleSupplier leftYSupplier,
        java.util.function.DoubleSupplier leftXSupplier,
        double maxSpeed,
        double maxAngularRate
    ) {
        super(
            // If fiducial is detected, run the approach command
            new FunctionalCommand(
                // Initialize
                () -> {
                    SmartDashboard.putBoolean("ApproachTag/Running", true);
                    System.out.println("ApproachToTagCommand started - Robot will face AprilTag!");
                },
                // Execute - continuously adjust rotation based on tx
                () -> {
                    // Get horizontal offset from Limelight (tx)
                    double tx = LimelightHelpers.getTX("limelight");
                    boolean hasTarget = LimelightHelpers.getTV("limelight");

                    // Get joystick inputs for driver control with deadband
                    double leftY = leftYSupplier.getAsDouble();
                    double leftX = leftXSupplier.getAsDouble();

                    // Apply deadband (10%)
                    if (Math.abs(leftY) < DEADBAND) leftY = 0.0;
                    if (Math.abs(leftX) < DEADBAND) leftX = 0.0;

                    double forwardSpeed = leftY * maxSpeed;
                    double strafeSpeed = leftX * maxSpeed;

                    if (hasTarget) {
                        // Calculate steering adjustment using proportional control
                        double headingError = tx;
                        double steeringAdjust = 0.0;

                        // Apply proportional control with minimum command
                        // This prevents the robot from getting stuck due to friction
                        if (Math.abs(headingError) > ERROR_THRESHOLD) {
                            steeringAdjust = KP * headingError;

                            // Add minimum command to overcome friction
                            if (headingError < 0) {
                                steeringAdjust -= MIN_COMMAND;
                            } else {
                                steeringAdjust += MIN_COMMAND;
                            }
                        } else {
                            // Very close to target, use only proportional
                            steeringAdjust = KP * headingError;
                        }

                        // Clamp steering to max angular rate
                        steeringAdjust = Math.max(-maxAngularRate, Math.min(maxAngularRate, steeringAdjust));

                        // Debug output
                        SmartDashboard.putNumber("ApproachTag/TX", tx);
                        SmartDashboard.putNumber("ApproachTag/HeadingError", headingError);
                        SmartDashboard.putNumber("ApproachTag/SteeringAdjust", steeringAdjust);
                        SmartDashboard.putNumber("ApproachTag/ForwardSpeed", forwardSpeed);
                        SmartDashboard.putNumber("ApproachTag/StrafeSpeed", strafeSpeed);

                        // Apply field-centric drive with automatic rotation adjustment
                        drivetrain.setControl(
                            new SwerveRequest.FieldCentric()
                                .withVelocityX(forwardSpeed)
                                .withVelocityY(strafeSpeed)
                                .withRotationalRate(steeringAdjust)
                                .withDriveRequestType(DriveRequestType.Velocity)
                        );
                    } else {
                        // No target - just pass through joystick control
                        drivetrain.setControl(
                            new SwerveRequest.FieldCentric()
                                .withVelocityX(forwardSpeed)
                                .withVelocityY(strafeSpeed)
                                .withRotationalRate(0)
                                .withDriveRequestType(DriveRequestType.Velocity)
                        );
                        SmartDashboard.putBoolean("ApproachTag/NoTarget", true);
                    }
                },
                // End - stop the drivetrain
                (interrupted) -> {
                    drivetrain.setControl(new SwerveRequest.Idle());
                    SmartDashboard.putBoolean("ApproachTag/Running", false);
                    System.out.println("ApproachToTagCommand ended! Interrupted: " + interrupted);
                },
                // IsFinished - never finish automatically
                // Command will run as long as the button is held
                () -> false,
                drivetrain
            ),
            // If no fiducial detected, do nothing
            new InstantCommand(),
            // Condition: check if fiducial is detected
            () -> LimelightHelpers.getFiducialID("limelight") != -1
        );
    }
}
