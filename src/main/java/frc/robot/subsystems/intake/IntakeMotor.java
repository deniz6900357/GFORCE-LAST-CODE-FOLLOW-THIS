package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.CAN;

/**
 * Intake motor subsystem using Kraken X60 motor.
 *
 * <p>Single Kraken X60 motor connected to CANivore for intake roller control.
 * Velocity control - voltajdan bağımsız sabit hız.
 */
@Logged
public class IntakeMotor extends SubsystemBase {
    private static final int MOTOR_CAN_ID = 31; // Intake motor CAN ID
    // 80% of 6000 RPM = 4800 RPM = 80 rot/s (503 rad/s)
    private static final double INTAKE_VELOCITY_RPS = 80.0; // rot/s
    private static final int CURRENT_LIMIT_AMPS = 60; // Kraken X60 safe limit

    private final TalonFX intakeMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    public IntakeMotor() {
        if (RobotBase.isReal()) {
            // Create Kraken X60 motor on CANivore bus
            intakeMotor = new TalonFX(MOTOR_CAN_ID, CAN.CANIVORE_BUS);

            // Configure motor
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT_AMPS;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            // Velocity PID - Slot 0: kV = 12V / 100 rot/s (Kraken X60 free speed)
            config.Slot0.kV = 0.12;
            config.Slot0.kP = 0.1;

            intakeMotor.getConfigurator().apply(config);
        } else {
            intakeMotor = null;
        }
    }

    /**
     * Sets the intake motor velocity in rot/s.
     */
    public void setVelocity(double velocityRPS) {
        if (intakeMotor != null) {
            intakeMotor.setControl(velocityRequest.withVelocity(velocityRPS));
        }
    }

    /**
     * Sets the intake motor output as duty cycle (0.0 - 1.0).
     */
    public void setDutyCycle(double percent) {
        if (intakeMotor != null) {
            intakeMotor.setControl(dutyCycleRequest.withOutput(percent));
        }
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        if (intakeMotor != null) {
            intakeMotor.stopMotor();
        }
    }

    /**
     * Creates a command to run intake at default velocity (45% = 45 rot/s).
     */
    public Command intakeCommand() {
        return startEnd(
            () -> setVelocity(INTAKE_VELOCITY_RPS),
            this::stop
        ).withName("Intake: Run");
    }

    /**
     * Creates a command to run intake at custom velocity in rot/s.
     */
    public Command intakeCommand(double velocityRPS) {
        return startEnd(
            () -> setVelocity(velocityRPS),
            this::stop
        ).withName("Intake: " + (int) velocityRPS + " rot/s");
    }

    /**
     * Creates a command to stop the intake.
     *
     * @return Command that stops intake motor
     */
    public Command stopCommand() {
        return runOnce(this::stop).withName("Intake: Stop");
    }
}

