package frc.robot.subsystems.feeder;

/**
 * Simulation implementation of FeederIO.
 * Provides basic simulation behavior without hardware dependencies.
 */
public class FeederIOSim implements FeederIO {

    private double currentSpeed = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.connected = true;
        inputs.velocityRPM = currentSpeed * 5000.0; // Simulate ~5000 RPM at full speed
        inputs.appliedVolts = currentSpeed * 12.0;
        inputs.supplyCurrentAmps = Math.abs(currentSpeed) * 20.0; // Simulate 20A at full speed
        inputs.tempCelsius = 25.0 + Math.abs(currentSpeed) * 10.0; // Simulate heating
    }

    @Override
    public void setSpeed(double speed) {
        this.currentSpeed = speed;
    }

    @Override
    public void stop() {
        this.currentSpeed = 0.0;
    }
}
