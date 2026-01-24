package frc.robot.subsystems.shooter.flywheel;

/**
 * Hardware abstraction interface for the shooter flywheel subsystem.
 *
 * <p>Based on Mechanical Advantage Team 6328 RobotCode2026Public.
 * This interface allows swapping between real hardware and simulation.
 */
public interface FlywheelIO {

    /**
     * Input data from the flywheel motor.
     * Contains all sensor readings and status information.
     */
    public static class FlywheelIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    /**
     * Output control modes for the flywheel.
     */
    public enum FlywheelIOOutputMode {
        /** Motor coasts (no brake, no drive) */
        COAST,

        /** Duty cycle control for initial velocity ramp */
        DUTY_CYCLE_BANG_BANG,

        /** Torque current control for precise velocity holding */
        TORQUE_CURRENT_BANG_BANG
    }

    /**
     * Output commands to the flywheel motor.
     */
    public static class FlywheelIOOutputs {
        public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
        public double velocityRadPerSec = 0.0;
    }

    /**
     * Updates the input data from hardware.
     *
     * @param inputs The input object to populate with current sensor data
     */
    default void updateInputs(FlywheelIOInputs inputs) {}

    /**
     * Applies the output commands to hardware.
     *
     * @param outputs The output commands to apply
     */
    default void applyOutputs(FlywheelIOOutputs outputs) {}
}
