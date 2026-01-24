package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

/**
 * Real hardware implementation of FeederIO using REV NEO motor.
 *
 * <p>Feeder uses NEO motor with 27:1 reduction, connected to RoboRIO CAN bus.
 * Uses REVLib 2025 API.
 */
public class FeederIOReal implements FeederIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    // Motor configuration
    private static final int MOTOR_CAN_ID = 30; // DEĞİŞTİRİN - feeder motor ID'niz
    private static final boolean INVERT_MOTOR = false; // Gerekirse değiştirin
    private static final int CURRENT_LIMIT_AMPS = 40; // NEO için güvenli limit

    /**
     * Creates a new FeederIOReal instance.
     */
    public FeederIOReal() {
        // Create NEO motor controller
        motor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Create configuration
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(INVERT_MOTOR);
        config.idleMode(IdleMode.kBrake); // Brake mode for quick stop
        config.smartCurrentLimit(CURRENT_LIMIT_AMPS);

        // Apply configuration
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // Check if motor is connected (REVLib 2025+ uses != OK)
        inputs.connected = motor.getLastError() == com.revrobotics.REVLibError.kOk;

        inputs.velocityRPM = encoder.getVelocity(); // RPM at motor shaft
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.supplyCurrentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setSpeed(double speed) {
        // Clamp speed to [-1.0, 1.0]
        speed = Math.max(-1.0, Math.min(1.0, speed));
        motor.set(speed);
    }

    @Override
    public void stop() {
        motor.set(0.0);
    }
}
