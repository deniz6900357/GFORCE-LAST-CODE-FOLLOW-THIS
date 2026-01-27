package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Feeder subsystem for note feeding control.
 *
 * <p>Uses a single NEO motor with 27:1 reduction.
 * Simple percent output control for feeding notes into shooter.
 */
public class Feeder extends SubsystemBase {

    private final FeederIO io;
    private final FeederIO.FeederIOInputs inputs = new FeederIO.FeederIOInputs();

    // Default feed speed
    private static final double DEFAULT_FEED_SPEED = 0.1; // 10% forward

    /**
     * Creates a new Feeder subsystem.
     *
     * @param io The IO implementation to use
     */
    public Feeder(FeederIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);

        // Telemetry
        updateTelemetry();
    }

    /**
     * Sets the feeder motor speed.
     *
     * @param speed Motor speed percentage (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    /**
     * Feeds notes forward at default speed.
     */
    public void feed() {
        io.setSpeed(DEFAULT_FEED_SPEED);
    }

    /**
     * Reverses feeder (ejects notes).
     */
    public void reverse() {
        io.setSpeed(-DEFAULT_FEED_SPEED);
    }

    /**
     * Stops the feeder.
     */
    public void stop() {
        io.stop();
    }

    /**
     * Creates a command to feed notes forward.
     *
     * @return Command that runs the feeder
     */
    public Command feedCommand() {
        return run(this::feed).withName("Feeder: Feed");
    }

    /**
     * Creates a command to feed notes forward with custom speed.
     *
     * @param speed Motor speed percentage (-1.0 to 1.0)
     * @return Command that runs the feeder at specified speed
     */
    public Command feedCommand(double speed) {
        return run(() -> setSpeed(speed)).withName("Feeder: " + (int)(speed * 100) + "%");
    }

    /**
     * Creates a command to reverse the feeder (eject).
     *
     * @return Command that reverses the feeder
     */
    public Command reverseCommand() {
        return run(this::reverse).withName("Feeder: Reverse");
    }

    /**
     * Creates a command to stop the feeder.
     *
     * @return Command that stops the feeder
     */
    public Command stopCommand() {
        return runOnce(this::stop).withName("Feeder: Stop");
    }

    /**
     * Updates telemetry to SmartDashboard.
     */
    private void updateTelemetry() {
        SmartDashboard.putNumber("Feeder/Velocity (RPM)", inputs.velocityRPM);
        SmartDashboard.putNumber("Feeder/Applied Volts", inputs.appliedVolts);
        SmartDashboard.putNumber("Feeder/Current (A)", inputs.supplyCurrentAmps);
        SmartDashboard.putNumber("Feeder/Temp (C)", inputs.tempCelsius);
        SmartDashboard.putBoolean("Feeder/Connected", inputs.connected);
    }
}
