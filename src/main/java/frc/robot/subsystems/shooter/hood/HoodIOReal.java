// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Real hardware implementation of HoodIO using CTRE Kraken X44 motor.
 *
 * <p>Uses Phoenix 6 Motion Magic for smooth position control with onboard PID.
 * Motion Magic handles acceleration, cruise velocity, and jerk limiting automatically.
 */
public class HoodIOReal implements HoodIO {

    private final TalonFX motor;

    // Control requests
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

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

        // Feedback sensor configuration - CRITICAL for gear ratio!
        config.Feedback.SensorToMechanismRatio = ShooterConstants.Hood.GEAR_RATIO;

        // Motion Magic configuration - Smooth and responsive
        config.MotionMagic.MotionMagicCruiseVelocity = 80.0;  // rot/s (mechanism rotations)
        config.MotionMagic.MotionMagicAcceleration = 160.0;   // rot/s²
        config.MotionMagic.MotionMagicJerk = 1600.0;          // rot/s³

        // Position PID gains - Will be updated from Hood.java via outputs
        // These are initial values, real values come from LoggedTunableNumber
        config.Slot0.kP = 100.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 5.0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = 0.2;  // Gravity compensation for arm mechanism

        motor.getConfigurator().apply(config);

        // Optimize bus utilization
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected = motor.isAlive();
        inputs.positionRads = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
        inputs.velocityRadsPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
        inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {
        switch (outputs.mode) {
            case BRAKE:
            case COAST:
                motor.setControl(neutralRequest);
                break;

            case OPEN_LOOP:
                motor.setControl(voltageRequest.withOutput(outputs.appliedVolts));
                break;

            case CLOSED_LOOP:
                // Convert radians to rotations for Motion Magic
                double targetPositionRotations = Units.radiansToRotations(outputs.positionRad);

                // Use Motion Magic for smooth position control
                // Motion Magic automatically handles acceleration, cruise velocity, and jerk
                motor.setControl(motionMagicRequest.withPosition(targetPositionRotations));
                break;
        }
    }
}
