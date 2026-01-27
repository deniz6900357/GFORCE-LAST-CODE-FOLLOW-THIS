package frc.robot.subsystems.shooter.hood;

/**
 * Simulation/stub implementation of HoodIO when hood hardware is not connected.
 *
 * <p>This implementation does nothing but prevents errors when hood is not physically attached.
 * Useful for testing other subsystems without hood hardware.
 */
public class HoodIOSim implements HoodIO {

    private double simulatedPositionRad = 0.0;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // Simulate hood as disconnected
        inputs.motorConnected = false;
        inputs.positionRad = simulatedPositionRad;
        inputs.velocityRadPerSec = 0.0;
        inputs.appliedVolts = 0.0;
        inputs.supplyCurrentAmps = 0.0;
        inputs.tempCelsius = 25.0; // Room temperature
    }

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {
        // In simulation, just track the commanded position
        if (outputs.mode == HoodIOOutputMode.CLOSED_LOOP) {
            simulatedPositionRad = outputs.positionRad;
        }
    }
}
