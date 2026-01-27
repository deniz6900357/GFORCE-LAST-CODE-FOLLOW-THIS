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
    public static final String CAN_BUS_NAME = "CANivore"; // Using CANivore bus

    /**
     * Flywheel constants - Uses 1x Kraken X60 motor.
     */
    public static final class Flywheel {
        // Motor CAN ID - CONFIGURE THIS FOR YOUR ROBOT
        public static final int MOTOR_ID = 35;

        // Motor inversion
        public static final boolean INVERT_MOTOR = false;

        // Velocity PID gains (tune these for your robot)
        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KV = 0.12; // Feedforward gain (volts per rad/s)
        public static final double KS = 0.2;  // Static friction compensation (volts)

        // Torque current control (for bang-bang mode)
        public static final double TORQUE_CURRENT_SCALE = 2.0; // Amps per rad/s

        // Velocity tolerance (rad/s)
        public static final double VELOCITY_TOLERANCE = 20.0; // ~3.2 RPS

        // Gear ratio (motor rotations : flywheel rotations)
        public static final double GEAR_RATIO = 1.0; // 1:1 direct drive

        // Preset velocities (rad/s) - matching GFORCE values
        public static final double IDLE_VELOCITY = 0.0;
        public static final double FENDER_VELOCITY = 200.0;   // Close shot
        public static final double MID_RANGE_VELOCITY = 350.0; // Medium distance
        public static final double FAR_VELOCITY = 450.0;      // Far shot
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

        // Angle limits (radians)
        public static final double MIN_ANGLE_RAD = Math.toRadians(19.0); // Most flat
        public static final double MAX_ANGLE_RAD = Math.toRadians(51.0); // Most lofted

        // Position tolerance (radians)
        public static final double POSITION_TOLERANCE = Math.toRadians(1.0); // ~1 degree

        // Gear ratio (motor rotations : hood rotations)
        // IMPORTANT: Measure this for your robot!
        public static final double GEAR_RATIO = 50.0; // Example: 50:1 reduction

        // Preset angles (radians) - tune based on testing
        public static final double FENDER_ANGLE = Math.toRadians(19.0);    // Close shot
        public static final double MID_RANGE_ANGLE = Math.toRadians(30.0); // Medium distance
        public static final double FAR_ANGLE = Math.toRadians(45.0);       // Far shot
        public static final double STOW_ANGLE = Math.toRadians(19.0);      // Stowed position
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
