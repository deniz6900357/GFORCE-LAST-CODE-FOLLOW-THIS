package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Real hardware implementation of FlywheelIO using single CTRE Kraken X60 motor.
 *
 * <p>Based on Mechanical Advantage Team 6328 RobotCode2026Public.
 * Uses Phoenix 6 API with bang-bang velocity control.
 */
public class FlywheelIOReal implements FlywheelIO {

    private final TalonFX motor;

    // Control requests
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0);

    /**
     * Creates a new FlywheelIOReal instance.
     */
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

        // Velocity PID configuration (optional, for future use)
        config.Slot0.kP = ShooterConstants.Flywheel.KP;
        config.Slot0.kI = ShooterConstants.Flywheel.KI;
        config.Slot0.kD = ShooterConstants.Flywheel.KD;
        config.Slot0.kV = ShooterConstants.Flywheel.KV;
        config.Slot0.kS = ShooterConstants.Flywheel.KS;

        motor.getConfigurator().apply(config);

        // Optimize bus utilization
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.connected = motor.isAlive();
        inputs.positionRad = motor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
        inputs.velocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
        inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(FlywheelIOOutputs outputs) {
        switch (outputs.mode) {
            case COAST:
                motor.setControl(dutyCycleRequest.withOutput(0.0));
                break;

            case DUTY_CYCLE_BANG_BANG:
                // Bang-bang control: full power when below setpoint
                double dutyCycle = outputs.velocityRadPerSec > 0.0 ? 1.0 : 0.0;
                motor.setControl(dutyCycleRequest.withOutput(dutyCycle));
                break;

            case TORQUE_CURRENT_BANG_BANG:
                // Torque current control: precise velocity holding near setpoint
                // Convert rad/s to rotations per second
                double velocityRPS = outputs.velocityRadPerSec / (2.0 * Math.PI);

                // Calculate feedforward torque current
                double feedforwardCurrent = velocityRPS * ShooterConstants.Flywheel.TORQUE_CURRENT_SCALE;

                motor.setControl(torqueCurrentRequest.withOutput(feedforwardCurrent));
                break;
        }
    }
}
