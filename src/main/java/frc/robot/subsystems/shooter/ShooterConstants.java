package frc.robot.subsystems.shooter;

/**
 * Constants for the hooded shooter subsystem.
 *
 * <p>Based on Mechanical Advantage Team 6328 RobotCode2026Public.
 * This robot has TWO motors:
 * - Kraken X60: Flywheel (shooter speed)
 * - Kraken X44: Hood (shooter angle)
 */
public final class ShooterConstants {

    private ShooterConstants() {
        throw new AssertionError("Utility class should not be instantiated");
    }

    /** CAN bus name for shooter motors */
    public static final String CAN_BUS_NAME = "CANivore"; // Change to "rio" if NOT using CANivore

    /**
     * Flywheel constants - Uses 1x Kraken X60 motor.
     */
    public static final class Flywheel {
        // Motor CAN ID - CONFIGURE THIS FOR YOUR ROBOT
        public static final int MOTOR_ID = 35;

        // Motor inversion
        public static final boolean INVERT_MOTOR = true;

        // Velocity PID gains (MA 6328 values)
        public static final double KP = 0.4;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KV = 0.019; // Feedforward gain (volts per rad/s)
        public static final double KS = 0.22;  // Static friction compensation (volts)

        // Slew rate limiter - max hızlanma (rad/s²)
        public static final double MAX_ACCELERATION = 250.0;

        // Gear ratio (motor rotations : flywheel rotations)
        public static final double GEAR_RATIO = 1.0; // 1:1 direct drive

        // Preset velocities (rad/s)
        public static final double IDLE_VELOCITY = 0.0;
        public static final double FENDER_VELOCITY = 235.0;   // Close shot
        public static final double MID_RANGE_VELOCITY = 276.0; // Medium distance
        public static final double FAR_VELOCITY = 330.0;       // Far shot
    }

    /**
     * Hood constants - Uses 1x Kraken X44 motor.
     */
    public static final class Hood {
        // Motor CAN ID - CONFIGURE THIS FOR YOUR ROBOT
        public static final int MOTOR_ID = 34;

        // Motor inversion
        public static final boolean INVERT_MOTOR = false;

        // Position PID gains (tune these for your robot)
        public static final double KP = 30000.0; // From MA simulation
        public static final double KI = 0.0;
        public static final double KD = 300.0;   // From MA simulation

        // Angle limits (radians) - MA 6328 values
        public static final double MIN_ANGLE_RAD = Math.toRadians(0.0); // Most flat
        public static final double MAX_ANGLE_RAD = Math.toRadians(12.0); // Most lofted

        // Position tolerance (radians)
        public static final double POSITION_TOLERANCE = Math.toRadians(1.0); // ~1 degree

        // Gear ratio (motor rotations : hood rotations)
        // IMPORTANT: Measure this for your robot!
        // Tuned based on 15°-45° test: needs 30° movement (was 18° at ratio 100)
        // NOTE: MA original was 80:1, but our robot's mechanism requires 166:1
        public static final double GEAR_RATIO = 166.0; // Motor rotations per mechanism rotation

        // Preset angles (radians)
        public static final double FENDER_ANGLE = Math.toRadians(0.0);     // Close shot
        public static final double MID_RANGE_ANGLE = Math.toRadians(6.0);   // Medium distance
        public static final double FAR_ANGLE = Math.toRadians(12.0);       // Far shot
        public static final double STOW_ANGLE = Math.toRadians(0.0);      // Stowed position
    }

    /**
     * Preset shooting configurations combining flywheel velocity and hood angle.
     *
     * <p>Based on Mechanical Advantage's approach.
     */
    public enum ShootingPreset {
        IDLE(Hood.STOW_ANGLE, Flywheel.IDLE_VELOCITY),
        FENDER(Hood.FENDER_ANGLE, Flywheel.FENDER_VELOCITY),
        MID_RANGE(Hood.MID_RANGE_ANGLE, Flywheel.MID_RANGE_VELOCITY),
        FAR(Hood.FAR_ANGLE, Flywheel.FAR_VELOCITY);

        public final double hoodAngleRad;
        public final double flywheelVelocityRadPerSec;

        ShootingPreset(double hoodAngleRad, double flywheelVelocityRadPerSec) {
            this.hoodAngleRad = hoodAngleRad;
            this.flywheelVelocityRadPerSec = flywheelVelocityRadPerSec;
        }
    }
}
