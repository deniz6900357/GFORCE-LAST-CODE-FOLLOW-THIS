package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Real hardware implementation of HoodIO using CTRE Kraken X44 motor.
 *
 * <p>Based on Mechanical Advantage Team 6328's architecture.
 * Uses Phoenix 6 Motion Magic for smooth position control.
 */
public class HoodIOReal implements HoodIO {

    private final TalonFX motor;

    // Control requests
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    /**
     * Creates a new HoodIOReal instance.
     */
    public HoodIOReal() {
        motor = new TalonFX(ShooterConstants.Hood.MOTOR_ID, ShooterConstants.CAN_BUS_NAME);

        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = ShooterConstants.Hood.INVERT_MOTOR
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        // Motion Magic configuration for smooth motion
        config.MotionMagic.MotionMagicCruiseVelocity = 80.0; // rot/s
        config.MotionMagic.MotionMagicAcceleration = 160.0;  // rot/s^2
        config.MotionMagic.MotionMagicJerk = 1600.0;         // rot/s^3

        // Position PID gains (will be overridden by outputs, but set defaults)
        config.Slot0.kP = ShooterConstants.Hood.KP;
        config.Slot0.kI = ShooterConstants.Hood.KI;
        config.Slot0.kD = ShooterConstants.Hood.KD;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = 0.0; // Tune this for gravity compensation

        // Feedback sensor configuration
        config.Feedback.SensorToMechanismRatio = ShooterConstants.Hood.GEAR_RATIO;

        motor.getConfigurator().apply(config);

        // Optimize bus utilization
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected = motor.isAlive();
        inputs.positionRad = motor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
        inputs.velocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {
        switch (outputs.mode) {
            case BRAKE:
            case COAST:
                motor.setControl(neutralRequest);
                break;

            case CLOSED_LOOP:
                // Convert radians to rotations for motor
                double positionRotations = outputs.positionRad / (2.0 * Math.PI);

                // Use Motion Magic for smooth position control
                // Feed-forward velocity is handled by Motion Magic internally
                motor.setControl(positionRequest.withPosition(positionRotations));
                break;
        }
    }
}
