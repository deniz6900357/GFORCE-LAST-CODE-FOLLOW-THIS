package frc.robot.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * AutoAlign - Automated tower scoring position alignment.
 *
 * <p>Provides odometry-based auto-alignment to tower scoring positions
 * using PathPlanner's pathfinding capabilities.
 * Limelight MegaTag2 continuously corrects odometry in the background.
 *
 * <p>Tower has two scoring lanes:
 * - Upper lane: Y = 4.0m (robot Y > 3.75 threshold)
 * - Lower lane: Y = 3.4m (robot Y <= 3.75 threshold)
 */
public class AutoAlign {

    // Y threshold to decide upper vs lower scoring position
    private static final double Y_THRESHOLD = 3.75;

    // Upper scoring position (robot Y > threshold)
    public static final Pose2d UPPER_SCORING_POSE = new Pose2d(1.5, 4.0, Rotation2d.fromDegrees(0));

    // Lower scoring position (robot Y <= threshold)
    public static final Pose2d LOWER_SCORING_POSE = new Pose2d(1.5, 3.4, Rotation2d.fromDegrees(0));

    private AutoAlign() {}

    /**
     * Creates a command that automatically aligns to the correct tower scoring position.
     * Chooses upper or lower lane based on robot's current Y position.
     *
     * @param drivetrain Drivetrain subsystem to get current robot position
     * @return Command that moves robot to the nearest scoring position
     */
    public static Command alignToTower(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            double robotY = drivetrain.getState().Pose.getY();
            Pose2d targetPose;

            if (robotY > Y_THRESHOLD) {
                targetPose = UPPER_SCORING_POSE;
                System.out.println("AutoAlign: Y > " + Y_THRESHOLD + " → Upper lane (1.5, 4.0)");
            } else {
                targetPose = LOWER_SCORING_POSE;
                System.out.println("AutoAlign: Y <= " + Y_THRESHOLD + " → Lower lane (1.5, 3.4)");
            }

            return AutoTrench.moveToPose(targetPose);
        }, java.util.Set.of(drivetrain)).withName("AutoAlign: Tower Scoring");
    }
}
