package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shot calculator for determining optimal shooter parameters based on distance.
 *
 * <p>Based on Mechanical Advantage Team 6328 RobotCode2026Public.
 * Uses interpolation tables to calculate hood angle and flywheel velocity
 * for different target distances.
 *
 * <p>This is a singleton class - use {@link #getInstance()} to access.
 */
public class ShotCalculator {
    private static ShotCalculator instance;

    // Hub/speaker positions (meters) - adjust for your field
    private static final Translation2d BLUE_SPEAKER = new Translation2d(0.0, 5.55);
    private static final Translation2d RED_SPEAKER = new Translation2d(16.54, 5.55);

    // Interpolation maps: distance (meters) -> value
    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();

    // Distance limits
    private static final double MIN_DISTANCE = 1.34; // meters
    private static final double MAX_DISTANCE = 5.60; // meters

    static {
        // Hood angle interpolation table (distance in meters -> angle)
        // TUNE THESE VALUES FOR YOUR ROBOT!
        shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
        shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
        shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
        shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
        shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
        shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
        shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
        shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

        // Flywheel speed interpolation table (distance in meters -> rad/s)
        // TUNE THESE VALUES FOR YOUR ROBOT!
        shotFlywheelSpeedMap.put(1.34, 210.0);
        shotFlywheelSpeedMap.put(1.78, 220.0);
        shotFlywheelSpeedMap.put(2.17, 220.0);
        shotFlywheelSpeedMap.put(2.81, 230.0);
        shotFlywheelSpeedMap.put(3.82, 250.0);
        shotFlywheelSpeedMap.put(4.09, 255.0);
        shotFlywheelSpeedMap.put(4.40, 260.0);
        shotFlywheelSpeedMap.put(4.77, 265.0);
        shotFlywheelSpeedMap.put(5.57, 275.0);
        shotFlywheelSpeedMap.put(5.60, 290.0);
    }

    /**
     * Shooting parameters for a given shot.
     */
    public static class ShootingParameters {
        public final boolean isValid;
        public final double hoodAngleRad;
        public final double flywheelVelocityRadPerSec;
        public final double distanceToTarget;

        public ShootingParameters(boolean isValid, double hoodAngleRad,
                                 double flywheelVelocityRadPerSec, double distanceToTarget) {
            this.isValid = isValid;
            this.hoodAngleRad = hoodAngleRad;
            this.flywheelVelocityRadPerSec = flywheelVelocityRadPerSec;
            this.distanceToTarget = distanceToTarget;
        }

        /** Invalid parameters (out of range) */
        public static ShootingParameters invalid() {
            return new ShootingParameters(false, 0.0, 0.0, 0.0);
        }
    }

    private ShotCalculator() {}

    /**
     * Gets the singleton instance.
     *
     * @return The ShotCalculator instance
     */
    public static ShotCalculator getInstance() {
        if (instance == null) {
            instance = new ShotCalculator();
        }
        return instance;
    }

    /**
     * Calculates shooting parameters for the current robot pose.
     *
     * @param robotPose Current robot pose on the field
     * @return Shooting parameters (hood angle, flywheel velocity)
     */
    public ShootingParameters calculateShot(Pose2d robotPose) {
        // Get speaker position based on alliance
        Translation2d speakerPosition = getSpeakerPosition();

        // Calculate distance to speaker
        double distance = robotPose.getTranslation().getDistance(speakerPosition);

        // Update telemetry
        SmartDashboard.putNumber("ShotCalc/Distance (m)", distance);
        SmartDashboard.putBoolean("ShotCalc/In Range",
            distance >= MIN_DISTANCE && distance <= MAX_DISTANCE);

        // Check if distance is in valid range
        if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
            SmartDashboard.putString("ShotCalc/Status", "OUT OF RANGE");
            return ShootingParameters.invalid();
        }

        // Interpolate hood angle and flywheel speed from tables
        double hoodAngleRad = shotHoodAngleMap.get(distance).getRadians();
        double flywheelVelocity = shotFlywheelSpeedMap.get(distance);

        // Update telemetry
        SmartDashboard.putNumber("ShotCalc/Hood Angle (deg)", Math.toDegrees(hoodAngleRad));
        SmartDashboard.putNumber("ShotCalc/Flywheel Velocity (rad/s)", flywheelVelocity);
        SmartDashboard.putString("ShotCalc/Status", "READY");

        return new ShootingParameters(true, hoodAngleRad, flywheelVelocity, distance);
    }

    /**
     * Calculates shooting parameters for a specific distance.
     *
     * @param distance Distance to target in meters
     * @return Shooting parameters (hood angle, flywheel velocity)
     */
    public ShootingParameters calculateShotForDistance(double distance) {
        // Check if distance is in valid range
        if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
            return ShootingParameters.invalid();
        }

        // Interpolate hood angle and flywheel speed from tables
        double hoodAngleRad = shotHoodAngleMap.get(distance).getRadians();
        double flywheelVelocity = shotFlywheelSpeedMap.get(distance);

        return new ShootingParameters(true, hoodAngleRad, flywheelVelocity, distance);
    }

    /**
     * Gets the speaker position based on current alliance.
     *
     * @return Speaker position on the field
     */
    private Translation2d getSpeakerPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Blue ? BLUE_SPEAKER : RED_SPEAKER;
    }

    /**
     * Gets the minimum valid shooting distance.
     *
     * @return Minimum distance in meters
     */
    public double getMinDistance() {
        return MIN_DISTANCE;
    }

    /**
     * Gets the maximum valid shooting distance.
     *
     * @return Maximum distance in meters
     */
    public double getMaxDistance() {
        return MAX_DISTANCE;
    }
}
