package frc.robot.automation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeMotor;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Automated scoring sequence for the FRC robot.
 *
 * <p>This utility class provides a command sequence that:
 * <ol>
 *   <li>Moves the intake lift to the target scoring position</li>
 *   <li>Waits until the setpoint is reached</li>
 *   <li>Ejects the game piece at the specified speed and duration</li>
 *   <li>Returns the lift to the HOME position</li>
 * </ol>
 */
public final class AutomatedScoring {

    // Constants for timing control
    private static final double SETPOINT_TIMEOUT_SECONDS = 2.5;
    private static final double POST_EJECT_DELAY_SECONDS = 0.1;

    /**
     * Private constructor to prevent instantiation of utility class.
     */
    private AutomatedScoring() {
        throw new AssertionError("Utility class should not be instantiated");
    }

    /**
     * Creates an automated scoring command sequence.
     *
     * <p>The sequence moves the intake to a target position, ejects the game piece,
     * and returns to home position automatically.
     *
     * @param targetPosition The target height position for scoring (e.g., INTAKE_POSITION)
     * @param liftSubsystem  The intake subsystem controlling vertical movement
     * @param rollerMotor    The intake motor controlling game piece in/out movement
     * @param ejectSpeed     The speed at which to eject the game piece (uses absolute value)
     * @param ejectDuration  The duration in seconds to run the ejection
     * @return A sequential command that executes the full scoring sequence
     */
    public static Command score(
            double targetPosition,
            IntakeSubsystem liftSubsystem,
            IntakeMotor rollerMotor,
            double ejectSpeed,
            double ejectDuration) {

        return new SequentialCommandGroup(
                // Move lift to target scoring position
                liftSubsystem.goToSetpointCommand(targetPosition),

                // Wait for lift to reach setpoint (with timeout for safety)
                new WaitUntilCommand(liftSubsystem::atSetpoint)
                        .withTimeout(SETPOINT_TIMEOUT_SECONDS),

                // Eject the game piece
                rollerMotor.intakeOutCommand(Math.abs(ejectSpeed))
                        .withTimeout(ejectDuration),

                // Small delay after ejection
                new WaitCommand(POST_EJECT_DELAY_SECONDS),

                // Return lift to home position
                liftSubsystem.goToSetpointCommand(IntakeConstants.HeightSetpoints.HOME)
        );
    }
}
