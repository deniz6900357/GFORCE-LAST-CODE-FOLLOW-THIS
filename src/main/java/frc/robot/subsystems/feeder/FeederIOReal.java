package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

/**
 * Real hardware implementation of FeederIO using 3x REV NEO motors.
 *
 * <p>Feeder uses 3x NEO motors with 27:1 reduction, connected to RoboRIO CAN bus.
 * Uses REVLib 2025 API.
 */
public class FeederIOReal implements FeederIO {

    private final SparkMax motor1;
    private final SparkMax motor2;
    private final SparkMax motor3;
    private final RelativeEncoder encoder;

    // Motor configuration
    private static final int MOTOR_1_CAN_ID = 34; // Feeder motor 1 CAN ID (RoboRIO bus)
    private static final int MOTOR_2_CAN_ID = 33; // Feeder motor 2 CAN ID (RoboRIO bus)
    private static final int MOTOR_3_CAN_ID = 36; // Feeder motor 3 CAN ID (RoboRIO bus)
    private static final boolean INVERT_MOTOR_1 = false; // Gerekirse değiştirin
    private static final boolean INVERT_MOTOR_2 = true; // Gerekirse değiştirin
    private static final boolean INVERT_MOTOR_3 = false; // Gerekirse değiştirin
    private static final int CURRENT_LIMIT_AMPS = 80; // NEO için limit

    /**
     * Creates a new FeederIOReal instance with 2 NEO motors.
     */
    public FeederIOReal() {
        // Create NEO motor controllers
        motor1 = new SparkMax(MOTOR_1_CAN_ID, MotorType.kBrushless);
        motor2 = new SparkMax(MOTOR_2_CAN_ID, MotorType.kBrushless);
        motor3 = new SparkMax(MOTOR_3_CAN_ID, MotorType.kBrushless);
        encoder = motor1.getEncoder(); // Use motor1's encoder for telemetry

        // Create configuration for motor 1
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.inverted(INVERT_MOTOR_1);
        config1.idleMode(IdleMode.kBrake);
        config1.smartCurrentLimit(CURRENT_LIMIT_AMPS);
        config1.openLoopRampRate(0.0);
        config1.closedLoopRampRate(0.0);
        config1.voltageCompensation(12.0);

        // Create configuration for motor 2
        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.inverted(INVERT_MOTOR_2);
        config2.idleMode(IdleMode.kBrake);
        config2.smartCurrentLimit(CURRENT_LIMIT_AMPS);
        config2.openLoopRampRate(0.0);
        config2.closedLoopRampRate(0.0);
        config2.voltageCompensation(12.0);

        // Create configuration for motor 3
        SparkMaxConfig config3 = new SparkMaxConfig();
        config3.inverted(INVERT_MOTOR_3);
        config3.idleMode(IdleMode.kBrake);
        config3.smartCurrentLimit(CURRENT_LIMIT_AMPS);
        config3.openLoopRampRate(0.0);
        config3.closedLoopRampRate(0.0);
        config3.voltageCompensation(12.0);

        // Apply configurations (reset to factory defaults first to clear old settings)
        motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor3.configure(config3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // Check if motors are connected (REVLib 2025+ uses != OK)
        inputs.connected = motor1.getLastError() == com.revrobotics.REVLibError.kOk &&
                          motor2.getLastError() == com.revrobotics.REVLibError.kOk &&
                          motor3.getLastError() == com.revrobotics.REVLibError.kOk;

        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVolts = motor1.getAppliedOutput() * motor1.getBusVoltage();
        inputs.supplyCurrentAmps = motor1.getOutputCurrent() + motor2.getOutputCurrent() + motor3.getOutputCurrent();
        inputs.tempCelsius = Math.max(Math.max(motor1.getMotorTemperature(), motor2.getMotorTemperature()), motor3.getMotorTemperature());
    }

    @Override
    public void setSpeed(double speed) {
        // Clamp speed to [-1.0, 1.0]
        speed = Math.max(-1.0, Math.min(1.0, speed));
        motor1.set(speed);
        motor2.set(speed);
        motor3.set(speed);
    }

    @Override
    public void stop() {
        motor1.set(0.0);
        motor2.set(0.0);
        motor3.set(0.0);
    }
}
