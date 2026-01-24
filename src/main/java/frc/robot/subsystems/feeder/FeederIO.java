package frc.robot.subsystems.feeder;

/**
 * Hardware abstraction interface for the feeder subsystem.
 *
 * <p>Feeder uses a single NEO motor with 27:1 reduction for note feeding.
 */
public interface FeederIO {

    /**
     * Input data from the feeder motor.
     */
    public static class FeederIOInputs {
        public boolean connected = false;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    /**
     * Updates the input data from hardware.
     *
     * @param inputs The input object to populate with current sensor data
     */
    default void updateInputs(FeederIOInputs inputs) {}

    /**
     * Sets the motor speed as a percentage (-1.0 to 1.0).
     *
     * @param speed Motor speed percentage
     */
    default void setSpeed(double speed) {}

    /**
     * Stops the motor.
     */
    default void stop() {}
}
