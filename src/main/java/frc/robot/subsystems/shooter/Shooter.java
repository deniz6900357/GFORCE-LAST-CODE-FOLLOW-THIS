package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;

/**
 * Shooter utility class for coordinating Flywheel and Hood subsystems.
 *
 * <p>Based on Mechanical Advantage Team 6328 RobotCode2026Public architecture.
 * Provides factory methods for creating coordinated shooting commands.
 *
 * <p>Note: Unlike a traditional subsystem, this is a utility class. Flywheel and Hood
 * are separate subsystems that can be controlled independently or together.
 */
public final class Shooter {

    private Shooter() {
        throw new AssertionError("Utility class should not be instantiated");
    }

    /**
     * Checks if both flywheel and hood are at their goals.
     *
     * @param flywheel The flywheel subsystem
     * @param hood The hood subsystem
     * @return True if both subsystems are ready to shoot
     */
    public static boolean readyToShoot(Flywheel flywheel, Hood hood) {
        return flywheel.atGoal() && hood.atGoal() && hood.isZeroed();
    }

    /**
     * Creates a command to run shooter at a specific preset configuration.
     *
     * @param flywheel The flywheel subsystem
     * @param hood The hood subsystem
     * @param preset The preset configuration to use
     * @return Command that sets both flywheel and hood
     */
    public static Command runPresetCommand(Flywheel flywheel, Hood hood,
                                          ShooterConstants.ShootingPreset preset) {
        return Commands.parallel(
            flywheel.runVelocityCommand(preset.flywheelVelocityRadPerSec),
            hood.setAngleCommand(preset.hoodAngleRad)
        ).withName("Shooter: " + preset.name());
    }

    /**
     * Creates a command to run shooter at custom velocity and angle.
     *
     * @param flywheel The flywheel subsystem
     * @param hood The hood subsystem
     * @param velocityRadPerSec Flywheel velocity in rad/s
     * @param angleRad Hood angle in radians
     * @return Command that sets both flywheel and hood
     */
    public static Command runCustomCommand(Flywheel flywheel, Hood hood,
                                          double velocityRadPerSec, double angleRad) {
        return Commands.parallel(
            flywheel.runVelocityCommand(velocityRadPerSec),
            hood.setAngleCommand(angleRad)
        ).withName("Shooter: Custom");
    }

    /**
     * Creates a command to run shooter using ShotCalculator for current pose.
     *
     * <p>Calculates optimal hood angle and flywheel velocity based on distance to target.
     *
     * @param flywheel The flywheel subsystem
     * @param hood The hood subsystem
     * @param robotPoseSupplier Supplier for current robot pose
     * @return Command that calculates and sets shooter parameters
     */
    public static Command runCalculatedShotCommand(Flywheel flywheel, Hood hood,
                                                   java.util.function.Supplier<Pose2d> robotPoseSupplier) {
        return Commands.run(() -> {
            ShotCalculator.ShootingParameters params =
                ShotCalculator.getInstance().calculateShot(robotPoseSupplier.get());

            if (params.isValid) {
                flywheel.setVelocity(params.flywheelVelocityRadPerSec);
                hood.setAngle(params.hoodAngleRad);
            }
        }, flywheel, hood).withName("Shooter: Auto Aim");
    }

    /**
     * Creates a command to prepare for shooting and wait until ready.
     *
     * @param flywheel The flywheel subsystem
     * @param hood The hood subsystem
     * @param preset The preset configuration to use
     * @return Command that prepares shooter and waits until ready
     */
    public static Command prepareToShootCommand(Flywheel flywheel, Hood hood,
                                               ShooterConstants.ShootingPreset preset) {
        return Commands.sequence(
            // Set flywheel and hood to target
            runPresetCommand(flywheel, hood, preset),

            // Wait until both are at goal
            Commands.waitUntil(() -> readyToShoot(flywheel, hood))
        ).withName("Shooter: Prepare " + preset.name());
    }

    /**
     * Creates a command to prepare for calculated shot and wait until ready.
     *
     * @param flywheel The flywheel subsystem
     * @param hood The hood subsystem
     * @param robotPoseSupplier Supplier for current robot pose
     * @return Command that calculates, prepares, and waits until ready
     */
    public static Command prepareCalculatedShotCommand(Flywheel flywheel, Hood hood,
                                                      java.util.function.Supplier<Pose2d> robotPoseSupplier) {
        return Commands.sequence(
            // Calculate and set parameters once
            Commands.runOnce(() -> {
                ShotCalculator.ShootingParameters params =
                    ShotCalculator.getInstance().calculateShot(robotPoseSupplier.get());

                if (params.isValid) {
                    flywheel.setVelocity(params.flywheelVelocityRadPerSec);
                    hood.setAngle(params.hoodAngleRad);
                }
            }, flywheel, hood),

            // Wait until both are at goal
            Commands.waitUntil(() -> readyToShoot(flywheel, hood))
        ).withName("Shooter: Prepare Auto Aim");
    }

    /**
     * Creates a command to idle the shooter (stow hood, stop flywheel).
     *
     * @param flywheel The flywheel subsystem
     * @param hood The hood subsystem
     * @return Command that idles the shooter
     */
    public static Command idleCommand(Flywheel flywheel, Hood hood) {
        return Commands.parallel(
            flywheel.stopCommand(),
            hood.stowCommand()
        ).withName("Shooter: Idle");
    }

    /**
     * Creates a command to stop the flywheel (hood keeps position).
     *
     * @param flywheel The flywheel subsystem
     * @return Command that stops the flywheel
     */
    public static Command stopFlywheelCommand(Flywheel flywheel) {
        return flywheel.stopCommand();
    }
}

