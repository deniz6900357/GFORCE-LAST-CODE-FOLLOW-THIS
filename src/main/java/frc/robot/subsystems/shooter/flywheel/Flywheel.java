package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Flywheel subsystem for shooter velocity control.
 *
 * <p>Based on Mechanical Advantage Team 6328 RobotCode2026Public.
 * Controls shooter flywheel velocity using dual-mode bang-bang control:
 * <ul>
 *   <li>Duty Cycle Bang-Bang: Initial velocity ramp-up</li>
 *   <li>Torque Current Bang-Bang: Precise holding near setpoint</li>
 * </ul>
 */
public class Flywheel extends SubsystemBase {

    private final FlywheelIO io;
    private final FlywheelIO.FlywheelIOInputs inputs = new FlywheelIO.FlywheelIOInputs();
    private final FlywheelIO.FlywheelIOOutputs outputs = new FlywheelIO.FlywheelIOOutputs();

    // Goal velocity (rad/s)
    private double goalVelocity = 0.0;

    // State tracking
    private int consecutiveLoopsAtGoal = 0;
    private static final int LOOPS_AT_GOAL_REQUIRED = 10; // 200ms at 50Hz

    /**
     * Creates a new Flywheel subsystem.
     *
     * @param io The IO implementation to use
     */
    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);

        // Determine control mode based on velocity error
        double velocityError = Math.abs(goalVelocity - inputs.velocityRadPerSec);

        if (goalVelocity == 0.0) {
            // Coast to stop
            outputs.mode = FlywheelIO.FlywheelIOOutputMode.COAST;
        } else if (velocityError > ShooterConstants.Flywheel.VELOCITY_TOLERANCE) {
            // Far from setpoint - use duty cycle for fast ramp
            outputs.mode = FlywheelIO.FlywheelIOOutputMode.DUTY_CYCLE_BANG_BANG;
        } else {
            // Near setpoint - use torque current for precision
            outputs.mode = FlywheelIO.FlywheelIOOutputMode.TORQUE_CURRENT_BANG_BANG;
        }

        outputs.velocityRadPerSec = goalVelocity;

        // Apply outputs to hardware
        io.applyOutputs(outputs);

        // Track "at goal" status
        if (velocityError <= ShooterConstants.Flywheel.VELOCITY_TOLERANCE && goalVelocity != 0.0) {
            consecutiveLoopsAtGoal++;
        } else {
            consecutiveLoopsAtGoal = 0;
        }

        // Telemetry
        updateTelemetry();
    }

    /**
     * Sets the target flywheel velocity.
     *
     * @param velocityRadPerSec Target velocity in radians per second
     */
    public void setVelocity(double velocityRadPerSec) {
        this.goalVelocity = velocityRadPerSec;
    }

    /**
     * Gets the current measured velocity.
     *
     * @return Current velocity in radians per second
     */
    public double getVelocity() {
        return inputs.velocityRadPerSec;
    }

    /**
     * Checks if flywheel is at goal velocity.
     *
     * @return True if velocity is within tolerance for required duration
     */
    public boolean atGoal() {
        return consecutiveLoopsAtGoal >= LOOPS_AT_GOAL_REQUIRED;
    }

    /**
     * Stops the flywheel.
     */
    public void stop() {
        setVelocity(0.0);
    }

    /**
     * Creates a command to run the flywheel at a fixed velocity.
     *
     * @param velocityRadPerSec Target velocity in radians per second
     * @return Command that runs the flywheel
     */
    public Command runVelocityCommand(double velocityRadPerSec) {
        return run(() -> setVelocity(velocityRadPerSec))
                .withName("Flywheel: " + Math.round(velocityRadPerSec) + " rad/s");
    }

    /**
     * Creates a command to stop the flywheel.
     *
     * @return Command that stops the flywheel
     */
    public Command stopCommand() {
        return runOnce(this::stop).withName("Flywheel: Stop");
    }

    /**
     * Creates a command to run at idle velocity.
     *
     * @return Command that idles the flywheel
     */
    public Command idleCommand() {
        return runVelocityCommand(ShooterConstants.Flywheel.IDLE_VELOCITY);
    }

    /**
     * Updates telemetry to SmartDashboard.
     */
    private void updateTelemetry() {
        SmartDashboard.putNumber("Flywheel/Goal Velocity (rad/s)", goalVelocity);
        SmartDashboard.putNumber("Flywheel/Measured Velocity (rad/s)", inputs.velocityRadPerSec);
        SmartDashboard.putNumber("Flywheel/Velocity Error (rad/s)",
                Math.abs(goalVelocity - inputs.velocityRadPerSec));
        SmartDashboard.putBoolean("Flywheel/At Goal", atGoal());
        SmartDashboard.putString("Flywheel/Control Mode", outputs.mode.toString());

        SmartDashboard.putNumber("Flywheel/Current (A)", inputs.supplyCurrentAmps);
        SmartDashboard.putNumber("Flywheel/Temp (C)", inputs.tempCelsius);
        SmartDashboard.putBoolean("Flywheel/Connected", inputs.connected);

        // Convert rad/s to RPM for easier interpretation
        double rpm = inputs.velocityRadPerSec * 60.0 / (2.0 * Math.PI);
        double goalRpm = goalVelocity * 60.0 / (2.0 * Math.PI);
        SmartDashboard.putNumber("Flywheel/RPM", rpm);
        SmartDashboard.putNumber("Flywheel/Goal RPM", goalRpm);
    }
}
