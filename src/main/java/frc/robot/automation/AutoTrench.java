package frc.robot.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * AutoTrench - Automated trench run positioning utility class.
 *
 * <p>Provides methods to automatically move the robot to specific field positions
 * using PathPlanner's pathfinding capabilities.
 * PathPlanner obstacle grid ile engelleri otomatik olarak dolaşır.
 */
public class AutoTrench {

    // Movement constraints for automated positioning
    private static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
        6.0, 6.0,
        Units.degreesToRadians(720), Units.degreesToRadians(1080)
    );

    /**
     * Creates a command to move the robot to a specific pose using PathPlanner pathfinding.
     */
    public static Command moveToPose(Pose2d pose) {
        return Commands.defer(() -> {
            System.out.println("AutoTrench: Pathfinding to: " + pose);
            return AutoBuilder.pathfindToPose(pose, DEFAULT_CONSTRAINTS);
        }, java.util.Set.of());
    }

    /**
     * Creates a command to move the robot to a specific pose with custom constraints.
     */
    public static Command moveToPose(Pose2d pose, PathConstraints constraints) {
        return Commands.defer(() -> {
            System.out.println("AutoTrench: Pathfinding to: " + pose + " with custom constraints");
            return AutoBuilder.pathfindToPose(pose, constraints);
        }, java.util.Set.of());
    }

    /**
     * Creates a sequence command that moves to multiple poses in order.
     */
    public static Command moveToPoseSequence(Pose2d... poses) {
        Command[] commands = new Command[poses.length];
        for (int i = 0; i < poses.length; i++) {
            commands[i] = moveToPose(poses[i]);
        }
        return Commands.sequence(commands);
    }

    // ========== Predefined Field Positions ==========

    // Tek hedef noktalar - PathPlanner obstacle grid engelleri otomatik dolaşır
    // BLUE: Sol trench hedef Y=7.45 (üst), Sağ trench hedef Y=0.65 (alt)
    // RED: X mirror edilir (16.518 - X), yön 180°

    // Blue alliance trench hedef noktaları (X bazlı: yakın ve uzak)
    public static final Pose2d BLUE_LEFT_TRENCH_NEAR = new Pose2d(5.7, 7.45, Rotation2d.fromDegrees(0));   // X < 4.0 ise
    public static final Pose2d BLUE_LEFT_TRENCH_FAR = new Pose2d(3.5, 7.45, Rotation2d.fromDegrees(0));    // X >= 4.0 ise
    public static final Pose2d BLUE_RIGHT_TRENCH_NEAR = new Pose2d(5.7, 0.65, Rotation2d.fromDegrees(0));  // X < 4.0 ise
    public static final Pose2d BLUE_RIGHT_TRENCH_FAR = new Pose2d(3.5, 0.65, Rotation2d.fromDegrees(0));   // X >= 4.0 ise

    // Red alliance trench hedef noktaları (mirrored, X bazlı)
    public static final Pose2d RED_LEFT_TRENCH_NEAR = new Pose2d(10.818, 0.65, Rotation2d.fromDegrees(180));  // X > 12.518 ise
    public static final Pose2d RED_LEFT_TRENCH_FAR = new Pose2d(13.018, 0.65, Rotation2d.fromDegrees(180));   // X <= 12.518 ise
    public static final Pose2d RED_RIGHT_TRENCH_NEAR = new Pose2d(10.818, 7.45, Rotation2d.fromDegrees(180)); // X > 12.518 ise
    public static final Pose2d RED_RIGHT_TRENCH_FAR = new Pose2d(13.018, 7.45, Rotation2d.fromDegrees(180));  // X <= 12.518 ise

    /*
     * ========== ESKİ 2-WAYPOINT SİSTEMİ (yorum satırı olarak saklandı) ==========
     *
     * public static final Pose2d BLUE_LEFT_TRENCH_POS_1 = new Pose2d(5.7, 7.45, Rotation2d.fromDegrees(0));
     * public static final Pose2d BLUE_LEFT_TRENCH_POS_2 = new Pose2d(3.5, 7.45, Rotation2d.fromDegrees(0));
     * public static final Pose2d BLUE_RIGHT_TRENCH_POS_1 = new Pose2d(5.7, 0.65, Rotation2d.fromDegrees(0));
     * public static final Pose2d BLUE_RIGHT_TRENCH_POS_2 = new Pose2d(3.5, 0.65, Rotation2d.fromDegrees(0));
     * public static final Pose2d RED_LEFT_TRENCH_POS_1 = new Pose2d(10.818, 0.65, Rotation2d.fromDegrees(180));
     * public static final Pose2d RED_LEFT_TRENCH_POS_2 = new Pose2d(13.018, 0.65, Rotation2d.fromDegrees(180));
     * public static final Pose2d RED_RIGHT_TRENCH_POS_1 = new Pose2d(10.818, 7.45, Rotation2d.fromDegrees(180));
     * public static final Pose2d RED_RIGHT_TRENCH_POS_2 = new Pose2d(13.018, 7.45, Rotation2d.fromDegrees(180));
     *
     * // Eski alliance-aware left trench (2 waypoint):
     * // if (isBlue) {
     * //     if (robotX < 4.0) return moveToPoseSequence(BLUE_LEFT_TRENCH_POS_2, BLUE_LEFT_TRENCH_POS_1);
     * //     else return moveToPoseSequence(BLUE_LEFT_TRENCH_POS_1, BLUE_LEFT_TRENCH_POS_2);
     * // } else {
     * //     if (robotX > 12.518) return moveToPoseSequence(RED_LEFT_TRENCH_POS_2, RED_LEFT_TRENCH_POS_1);
     * //     else return moveToPoseSequence(RED_LEFT_TRENCH_POS_1, RED_LEFT_TRENCH_POS_2);
     * // }
     *
     * // Eski alliance-aware right trench (2 waypoint):
     * // if (isBlue) {
     * //     if (robotX < 4.0) return moveToPoseSequence(BLUE_RIGHT_TRENCH_POS_2, BLUE_RIGHT_TRENCH_POS_1);
     * //     else return moveToPoseSequence(BLUE_RIGHT_TRENCH_POS_1, BLUE_RIGHT_TRENCH_POS_2);
     * // } else {
     * //     if (robotX > 12.518) return moveToPoseSequence(RED_RIGHT_TRENCH_POS_2, RED_RIGHT_TRENCH_POS_1);
     * //     else return moveToPoseSequence(RED_RIGHT_TRENCH_POS_1, RED_RIGHT_TRENCH_POS_2);
     * // }
     */

    /**
     * Alliance-aware sol trench komutu (POV Left / LB).
     * Tek hedef noktaya gider, PathPlanner obstacle grid ile engelleri otomatik dolaşır.
     */
    public static Command allianceAwareLeftTrenchSequence(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance()
                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
            boolean isBlue = alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
            double robotX = drivetrain.getState().Pose.getX();

            Pose2d target;
            if (isBlue) {
                target = (robotX < 4.0) ? BLUE_LEFT_TRENCH_NEAR : BLUE_LEFT_TRENCH_FAR;
            } else {
                target = (robotX > 12.518) ? RED_LEFT_TRENCH_NEAR : RED_LEFT_TRENCH_FAR;
            }
            System.out.println("AutoTrench SOL: " + (isBlue ? "BLUE" : "RED") + " X=" + robotX + " -> " + target);
            return moveToPose(target);
        }, java.util.Set.of(drivetrain))
        .withName("Alliance-Aware Left Trench");
    }

    /**
     * Alliance-aware sağ trench komutu (POV Right / RB).
     * X bazlı hedef seçimi: robot yakınsa yakın hedefe, uzaksa uzak hedefe gider.
     */
    public static Command allianceAwareRightTrenchSequence(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance()
                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
            boolean isBlue = alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
            double robotX = drivetrain.getState().Pose.getX();

            Pose2d target;
            if (isBlue) {
                target = (robotX < 4.0) ? BLUE_RIGHT_TRENCH_NEAR : BLUE_RIGHT_TRENCH_FAR;
            } else {
                target = (robotX > 12.518) ? RED_RIGHT_TRENCH_NEAR : RED_RIGHT_TRENCH_FAR;
            }
            System.out.println("AutoTrench SAG: " + (isBlue ? "BLUE" : "RED") + " X=" + robotX + " -> " + target);
            return moveToPose(target);
        }, java.util.Set.of(drivetrain))
        .withName("Alliance-Aware Right Trench");
    }
}
