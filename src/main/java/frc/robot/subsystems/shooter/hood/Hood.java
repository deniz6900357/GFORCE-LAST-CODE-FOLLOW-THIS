package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Hood subsystem for shooter angle adjustment.
 *
 * <p>Based on Mechanical Advantage Team 6328's architecture.
 * Controls hood angle using closed-loop position control with Motion Magic.
 */
public class Hood extends SubsystemBase {

    private final HoodIO io;
    private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();
    private final HoodIO.HoodIOOutputs outputs = new HoodIO.HoodIOOutputs();

    // Goal angle (radians)
    private double goalAngleRad = ShooterConstants.Hood.STOW_ANGLE;

    // Zeroing state
    private boolean isZeroed = false;
    private double hoodOffset = 0.0; // Calibration offset

    // State tracking
    private int consecutiveLoopsAtGoal = 0;
    private static final int LOOPS_AT_GOAL_REQUIRED = 10; // 200ms at 50Hz

    /**
     * Creates a new Hood subsystem.
     *
     * @param io The IO implementation to use
     */
    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);

        // Determine control mode
        if (!isZeroed) {
            outputs.mode = HoodIO.HoodIOOutputMode.BRAKE;
        } else {
            outputs.mode = HoodIO.HoodIOOutputMode.CLOSED_LOOP;

            // Clamp goal angle to safe limits
            double clampedGoal = MathUtil.clamp(
                    goalAngleRad,
                    ShooterConstants.Hood.MIN_ANGLE_RAD,
                    ShooterConstants.Hood.MAX_ANGLE_RAD);

            // Apply offset calibration
            outputs.positionRad = clampedGoal - hoodOffset;
            outputs.kP = ShooterConstants.Hood.KP;
            outputs.kD = ShooterConstants.Hood.KD;
        }

        // Apply outputs to hardware
        io.applyOutputs(outputs);

        // Track "at goal" status
        double positionError = Math.abs(getMeasuredAngleRad() - goalAngleRad);
        if (isZeroed && positionError <= ShooterConstants.Hood.POSITION_TOLERANCE) {
            consecutiveLoopsAtGoal++;
        } else {
            consecutiveLoopsAtGoal = 0;
        }

        // Telemetry
        updateTelemetry();
    }

    /**
     * Zeros/calibrates the hood at its current position as the minimum angle.
     * Call this when the hood is physically at the minimum mechanical stop.
     */
    public void zero() {
        hoodOffset = ShooterConstants.Hood.MIN_ANGLE_RAD - inputs.positionRad;
        isZeroed = true;
        System.out.println("Hood zeroed! Offset: " + Math.toDegrees(hoodOffset) + " degrees");
    }

    /**
     * Sets the target hood angle.
     *
     * @param angleRad Target angle in radians
     */
    public void setAngle(double angleRad) {
        this.goalAngleRad = angleRad;
    }

    /**
     * Gets the current measured angle (with calibration applied).
     *
     * @return Current angle in radians
     */
    public double getMeasuredAngleRad() {
        return inputs.positionRad + hoodOffset;
    }

    /**
     * Checks if hood is at goal angle.
     *
     * @return True if angle is within tolerance for required duration
     */
    public boolean atGoal() {
        return isZeroed && consecutiveLoopsAtGoal >= LOOPS_AT_GOAL_REQUIRED;
    }

    /**
     * Checks if hood has been zeroed/calibrated.
     *
     * @return True if zeroed
     */
    public boolean isZeroed() {
        return isZeroed;
    }

    /**
     * Creates a command to zero the hood.
     * Run this when the hood is at its minimum mechanical stop.
     *
     * @return Command that zeros the hood
     */
    public Command zeroCommand() {
        return runOnce(this::zero)
                .ignoringDisable(true)
                .withName("Hood: Zero");
    }

    /**
     * Creates a command to run the hood at a fixed angle.
     *
     * @param angleRad Target angle in radians
     * @return Command that controls the hood angle
     */
    public Command setAngleCommand(double angleRad) {
        return run(() -> setAngle(angleRad))
                .withName("Hood: " + Math.round(Math.toDegrees(angleRad)) + "°");
    }

    /**
     * Creates a command to move to stow position.
     *
     * @return Command that stows the hood
     */
    public Command stowCommand() {
        return setAngleCommand(ShooterConstants.Hood.STOW_ANGLE);
    }

    /**
     * Updates telemetry to SmartDashboard.
     */
    private void updateTelemetry() {
        SmartDashboard.putNumber("Hood/Goal Angle (deg)", Math.toDegrees(goalAngleRad));
        SmartDashboard.putNumber("Hood/Measured Angle (deg)", Math.toDegrees(getMeasuredAngleRad()));
        SmartDashboard.putNumber("Hood/Raw Position (deg)", Math.toDegrees(inputs.positionRad));
        SmartDashboard.putNumber("Hood/Offset (deg)", Math.toDegrees(hoodOffset));

        double angleError = Math.abs(goalAngleRad - getMeasuredAngleRad());
        SmartDashboard.putNumber("Hood/Angle Error (deg)", Math.toDegrees(angleError));
        SmartDashboard.putBoolean("Hood/At Goal", atGoal());
        SmartDashboard.putBoolean("Hood/Zeroed", isZeroed);

        SmartDashboard.putNumber("Hood/Current (A)", inputs.supplyCurrentAmps);
        SmartDashboard.putNumber("Hood/Temp (C)", inputs.tempCelsius);
        SmartDashboard.putBoolean("Hood/Connected", inputs.motorConnected);
        SmartDashboard.putString("Hood/Control Mode", outputs.mode.toString());
    }
}
