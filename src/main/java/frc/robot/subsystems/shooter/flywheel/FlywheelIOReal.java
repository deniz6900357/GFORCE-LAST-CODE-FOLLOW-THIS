package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Real hardware implementation of FlywheelIO using single CTRE Kraken X60 motor.
 *
 * <p>Based on Mechanical Advantage Team 6328 RobotCode2026Public.
 * Uses Phoenix 6 VelocityVoltage control with feedforward.
 */
public class FlywheelIOReal implements FlywheelIO {

    private final TalonFX motor;

    // Control requests
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    // Track last PID values to avoid unnecessary config updates
    private double lastKP = -1;
    private double lastKD = -1;

    public FlywheelIOReal() {
        motor = new TalonFX(ShooterConstants.Flywheel.MOTOR_ID, ShooterConstants.CAN_BUS_NAME);

        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = ShooterConstants.Flywheel.INVERT_MOTOR
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        // Velocity PID gains (initial values, updated at runtime from Flywheel.java)
        config.Slot0.kP = ShooterConstants.Flywheel.KP;
        config.Slot0.kI = ShooterConstants.Flywheel.KI;
        config.Slot0.kD = ShooterConstants.Flywheel.KD;
        config.Slot0.kV = 0.0; // Feedforward Flywheel.java'da hesaplanıyor
        config.Slot0.kS = 0.0; // Feedforward Flywheel.java'da hesaplanıyor

        motor.getConfigurator().apply(config);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.connected = motor.isAlive();
        inputs.followerConnected = true;
        inputs.positionRads = motor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
        inputs.velocityRadsPerSec = motor.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
        inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
        inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
        inputs.followerSupplyCurrentAmps = 0.0;
        inputs.followerTempCelsius = 0.0;
    }

    @Override
    public void applyOutputs(FlywheelIOOutputs outputs) {
        // PID gains değiştiyse motora uygula
        if (outputs.kP != lastKP || outputs.kD != lastKD) {
            var slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
            slot0.kP = outputs.kP;
            slot0.kI = 0.0;
            slot0.kD = outputs.kD;
            slot0.kV = 0.0;
            slot0.kS = 0.0;
            motor.getConfigurator().apply(slot0);
            lastKP = outputs.kP;
            lastKD = outputs.kD;
        }

        switch (outputs.mode) {
            case COAST:
                motor.setControl(dutyCycleRequest.withOutput(0.0));
                break;

            case VELOCITY:
                // rad/s → RPS (rotations per second) for Phoenix 6
                double velocityRPS = outputs.velocityRadsPerSec / (2.0 * Math.PI);
                motor.setControl(velocityRequest
                        .withVelocity(velocityRPS)
                        .withFeedForward(outputs.feedforward));
                break;
        }
    }
}
