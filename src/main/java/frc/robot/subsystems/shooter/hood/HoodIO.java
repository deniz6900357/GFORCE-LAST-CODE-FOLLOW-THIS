package frc.robot.subsystems.shooter.hood;

/**
 * Hardware abstraction interface for the shooter hood subsystem.
 *
 * <p>Based on Mechanical Advantage Team 6328's architecture.
 * This interface allows swapping between real hardware and simulation.
 */
public interface HoodIO {

    /**
     * Input data from the hood motor.
     */
    public static class HoodIOInputs {
        public boolean motorConnected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    /**
     * Output control modes for the hood.
     */
    public enum HoodIOOutputMode {
        /** Motor brakes to hold position */
        BRAKE,

        /** Motor coasts (no brake, no drive) */
        COAST,

        /** Closed-loop position control */
        CLOSED_LOOP
    }

    /**
     * Output commands to the hood motor.
     */
    public static class HoodIOOutputs {
        public HoodIOOutputMode mode = HoodIOOutputMode.BRAKE;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double kP = 0.0;
        public double kD = 0.0;
    }

    /**
     * Updates the input data from hardware.
     *
     * @param inputs The input object to populate with current sensor data
     */
    default void updateInputs(HoodIOInputs inputs) {}

    /**
     * Applies the output commands to hardware.
     *
     * @param outputs The output commands to apply
     */
    default void applyOutputs(HoodIOOutputs outputs) {}
}
