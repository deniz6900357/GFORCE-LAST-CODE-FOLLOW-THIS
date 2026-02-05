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
 */
public class AutoTrench {

    // Movement constraints for automated positioning
    // AGGRESSIVE: Max velocity: 6.0 m/s, Max acceleration: 6.0 m/s² (çok hızlı ivmelenme!)
    // Max angular velocity: 720°/s, Max angular acceleration: 1080°/s² (çok hızlı dönüş!)
    private static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
        6.0, 6.0,
        Units.degreesToRadians(720), Units.degreesToRadians(1080)
    );

    /**
     * Creates a command to move the robot to a specific pose using PathPlanner pathfinding.
     *
     * @param pose Target pose (position and rotation) on the field
     * @return Command that moves robot to target pose
     */
    public static Command moveToPose(Pose2d pose) {
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            pose,
            DEFAULT_CONSTRAINTS
        );
        return Commands.deferredProxy(() -> pathfindingCommand);
    }

    /**
     * Creates a command to move the robot to a specific pose with custom constraints.
     *
     * @param pose Target pose (position and rotation) on the field
     * @param constraints Custom path constraints (velocity, acceleration limits)
     * @return Command that moves robot to target pose
     */
    public static Command moveToPose(Pose2d pose, PathConstraints constraints) {
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            pose,
            constraints
        );
        return Commands.deferredProxy(() -> pathfindingCommand);
    }

    /**
     * Creates a sequence command that moves to multiple poses in order.
     *
     * @param poses Array of target poses to visit in sequence
     * @return Command that visits all poses in order
     */
    public static Command moveToPoseSequence(Pose2d... poses) {
        Command[] commands = new Command[poses.length];
        for (int i = 0; i < poses.length; i++) {
            commands[i] = moveToPose(poses[i]);
        }
        return Commands.sequence(commands);
    }

    // ========== Predefined Field Positions ==========

    // BLUE ALLIANCE için:
    // - Y=7.45 (üst) = SOL trench (Button 7 - LB)
    // - Y=0.65 (alt) = SAĞ trench (Button 8 - RB)

    // RED ALLIANCE için (X mirror ediliyor):
    // - Y=7.45 (üst) = SAĞ trench (Button 8 - RB)
    // - Y=0.65 (alt) = SOL trench (Button 7 - LB)

    /**
     * Blue alliance LEFT trench position 1 (X: 5.7m, Y: 7.45m - ÜST, facing forward 0°)
     * Red alliance'da bu SAĞ trench olur
     */
    public static final Pose2d BLUE_LEFT_TRENCH_POS_1 = new Pose2d(5.7, 7.45, Rotation2d.fromDegrees(0));

    /**
     * Blue alliance LEFT trench position 2 (X: 3.5m, Y: 7.45m - ÜST, facing forward 0°)
     * Red alliance'da bu SAĞ trench olur
     */
    public static final Pose2d BLUE_LEFT_TRENCH_POS_2 = new Pose2d(3.5, 7.45, Rotation2d.fromDegrees(0));

    /**
     * Blue alliance RIGHT trench position 1 (X: 5.7m, Y: 0.65m - ALT, facing forward 0°)
     * Red alliance'da bu SOL trench olur
     */
    public static final Pose2d BLUE_RIGHT_TRENCH_POS_1 = new Pose2d(5.7, 0.65, Rotation2d.fromDegrees(0));

    /**
     * Blue alliance RIGHT trench position 2 (X: 3.5m, Y: 0.65m - ALT, facing forward 0°)
     * Red alliance'da bu SOL trench olur
     */
    public static final Pose2d BLUE_RIGHT_TRENCH_POS_2 = new Pose2d(3.5, 0.65, Rotation2d.fromDegrees(0));

    /**
     * Red alliance LEFT trench position 1 (mirrored, Y: 0.65m - ALT)
     * Field width: 16.518m, so X mirrored = 16.518 - 5.7 = 10.818m
     */
    public static final Pose2d RED_LEFT_TRENCH_POS_1 = new Pose2d(10.818, 0.65, Rotation2d.fromDegrees(180));

    /**
     * Red alliance LEFT trench position 2 (mirrored, Y: 0.65m - ALT)
     * Field width: 16.518m, so X mirrored = 16.518 - 3.5 = 13.018m
     */
    public static final Pose2d RED_LEFT_TRENCH_POS_2 = new Pose2d(13.018, 0.65, Rotation2d.fromDegrees(180));

    /**
     * Red alliance RIGHT trench position 1 (mirrored, Y: 7.45m - ÜST)
     * Field width: 16.518m, so X mirrored = 16.518 - 5.7 = 10.818m
     */
    public static final Pose2d RED_RIGHT_TRENCH_POS_1 = new Pose2d(10.818, 7.45, Rotation2d.fromDegrees(180));

    /**
     * Red alliance RIGHT trench position 2 (mirrored, Y: 7.45m - ÜST)
     * Field width: 16.518m, so X mirrored = 16.518 - 3.5 = 13.018m
     */
    public static final Pose2d RED_RIGHT_TRENCH_POS_2 = new Pose2d(13.018, 7.45, Rotation2d.fromDegrees(180));

    /**
     * Creates a command to move through blue alliance left trench sequence (Y: 7.45 - ÜST).
     * Goes to position 1 (5.7, 7.45) then position 2 (3.5, 7.45).
     *
     * @return Command that executes blue left trench sequence
     */
    public static Command blueLeftTrenchSequence() {
        return moveToPoseSequence(BLUE_LEFT_TRENCH_POS_1, BLUE_LEFT_TRENCH_POS_2)
            .withName("Blue Left Trench Sequence");
    }

    /**
     * Creates a command to move through blue alliance right trench sequence (Y: 0.65 - ALT).
     * Goes to position 1 (5.7, 0.65) then position 2 (3.5, 0.65).
     *
     * @return Command that executes blue right trench sequence
     */
    public static Command blueRightTrenchSequence() {
        return moveToPoseSequence(BLUE_RIGHT_TRENCH_POS_1, BLUE_RIGHT_TRENCH_POS_2)
            .withName("Blue Right Trench Sequence");
    }

    /**
     * Creates a command to move through red alliance left trench sequence (Y: 0.65 - ALT, mirrored).
     * Goes to position 1 (10.818, 0.65) then position 2 (13.018, 0.65).
     *
     * @return Command that executes red left trench sequence
     */
    public static Command redLeftTrenchSequence() {
        return moveToPoseSequence(RED_LEFT_TRENCH_POS_1, RED_LEFT_TRENCH_POS_2)
            .withName("Red Left Trench Sequence");
    }

    /**
     * Creates a command to move through red alliance right trench sequence (Y: 7.45 - ÜST, mirrored).
     * Goes to position 1 (10.818, 7.45) then position 2 (13.018, 7.45).
     *
     * @return Command that executes red right trench sequence
     */
    public static Command redRightTrenchSequence() {
        return moveToPoseSequence(RED_RIGHT_TRENCH_POS_1, RED_RIGHT_TRENCH_POS_2)
            .withName("Red Right Trench Sequence");
    }

    /**
     * Creates an alliance-aware left trench sequence command (Button 7 - LB).
     * Blue: Upper trench (Y: 7.45 - SOL)
     * Red: Lower trench (Y: 0.65 - SOL, mirrored)
     * SMART: If robot X < 4.0, reverses path direction (goes from 3.5 -> 5.7 instead)
     *
     * @param drivetrain Drivetrain subsystem to get current robot position
     * @return Command that executes appropriate left trench sequence for current alliance
     */
    public static Command allianceAwareLeftTrenchSequence(CommandSwerveDrivetrain drivetrain) {
        return Commands.either(
            // Blue alliance
            Commands.defer(() -> {
                double robotX = drivetrain.getState().Pose.getX();
                if (robotX < 4.0) {
                    // Robot sahada geriden, ters yönden git (3.5 -> 5.7)
                    System.out.println("🔄 Robot X < 4.0, TERS yön: 3.5 -> 5.7");
                    return moveToPoseSequence(BLUE_LEFT_TRENCH_POS_2, BLUE_LEFT_TRENCH_POS_1);
                } else {
                    // Robot sahada önden, normal yön (5.7 -> 3.5)
                    System.out.println("➡️ Robot X >= 4.0, NORMAL yön: 5.7 -> 3.5");
                    return moveToPoseSequence(BLUE_LEFT_TRENCH_POS_1, BLUE_LEFT_TRENCH_POS_2);
                }
            }, new java.util.HashSet<>()),
            // Red alliance
            Commands.defer(() -> {
                double robotX = drivetrain.getState().Pose.getX();
                if (robotX > 12.518) {
                    // Robot Red tarafında geriden, ters yönden git (13.018 -> 10.818)
                    System.out.println("🔄 Robot X > 12.518, TERS yön: 13.018 -> 10.818");
                    return moveToPoseSequence(RED_LEFT_TRENCH_POS_2, RED_LEFT_TRENCH_POS_1);
                } else {
                    // Robot Red tarafında önden, normal yön (10.818 -> 13.018)
                    System.out.println("➡️ Robot X <= 12.518, NORMAL yön: 10.818 -> 13.018");
                    return moveToPoseSequence(RED_LEFT_TRENCH_POS_1, RED_LEFT_TRENCH_POS_2);
                }
            }, new java.util.HashSet<>()),
            () -> {
                var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance()
                    .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
                return alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
            }
        ).withName("Alliance-Aware Left Trench Sequence")
         .beforeStarting(() -> System.out.println("🚀 AutoTrench SOL BAŞLADI!"))
         .andThen(() -> System.out.println("✅ AutoTrench SOL TAMAMLANDI!"));
    }

    /**
     * Creates an alliance-aware right trench sequence command (Button 8 - RB).
     * Blue: Lower trench (Y: 0.65 - SAĞ)
     * Red: Upper trench (Y: 7.45 - SAĞ, mirrored)
     * SMART: If robot X < 4.0, reverses path direction (goes from 3.5 -> 5.7 instead)
     *
     * @param drivetrain Drivetrain subsystem to get current robot position
     * @return Command that executes appropriate right trench sequence for current alliance
     */
    public static Command allianceAwareRightTrenchSequence(CommandSwerveDrivetrain drivetrain) {
        return Commands.either(
            // Blue alliance
            Commands.defer(() -> {
                double robotX = drivetrain.getState().Pose.getX();
                if (robotX < 4.0) {
                    // Robot sahada geriden, ters yönden git (3.5 -> 5.7)
                    System.out.println("🔄 Robot X < 4.0, TERS yön: 3.5 -> 5.7");
                    return moveToPoseSequence(BLUE_RIGHT_TRENCH_POS_2, BLUE_RIGHT_TRENCH_POS_1);
                } else {
                    // Robot sahada önden, normal yön (5.7 -> 3.5)
                    System.out.println("➡️ Robot X >= 4.0, NORMAL yön: 5.7 -> 3.5");
                    return moveToPoseSequence(BLUE_RIGHT_TRENCH_POS_1, BLUE_RIGHT_TRENCH_POS_2);
                }
            }, new java.util.HashSet<>()),
            // Red alliance
            Commands.defer(() -> {
                double robotX = drivetrain.getState().Pose.getX();
                if (robotX > 12.518) {
                    // Robot Red tarafında geriden, ters yönden git (13.018 -> 10.818)
                    System.out.println("🔄 Robot X > 12.518, TERS yön: 13.018 -> 10.818");
                    return moveToPoseSequence(RED_RIGHT_TRENCH_POS_2, RED_RIGHT_TRENCH_POS_1);
                } else {
                    // Robot Red tarafında önden, normal yön (10.818 -> 13.018)
                    System.out.println("➡️ Robot X <= 12.518, NORMAL yön: 10.818 -> 13.018");
                    return moveToPoseSequence(RED_RIGHT_TRENCH_POS_1, RED_RIGHT_TRENCH_POS_2);
                }
            }, new java.util.HashSet<>()),
            () -> {
                var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance()
                    .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
                return alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
            }
        ).withName("Alliance-Aware Right Trench Sequence")
         .beforeStarting(() -> System.out.println("🚀 AutoTrench SAĞ BAŞLADI!"))
         .andThen(() -> System.out.println("✅ AutoTrench SAĞ TAMAMLANDI!"));
    }
}
