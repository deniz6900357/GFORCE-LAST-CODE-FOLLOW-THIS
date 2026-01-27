package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.CAN;

/**
 * Intake motor subsystem using Kraken X60 motor.
 *
 * <p>Single Kraken X60 motor connected to CANivore for intake roller control.
 * Simple percent output control - no position tracking needed.
 */
@Logged
public class IntakeMotor extends SubsystemBase {
    private static final int MOTOR_CAN_ID = 31; // Intake motor CAN ID
    private static final double INTAKE_SPEED = 0.1; // 10% speed for intake
    private static final int CURRENT_LIMIT_AMPS = 60; // Kraken X60 safe limit

    private final TalonFX intakeMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    public IntakeMotor() {
        if (RobotBase.isReal()) {
            // Create Kraken X60 motor on CANivore bus
            intakeMotor = new TalonFX(MOTOR_CAN_ID, CAN.CANIVORE_BUS);

            // Configure motor
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT_AMPS;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast mode for easy hand-feeding

            intakeMotor.getConfigurator().apply(config);
        } else {
            intakeMotor = null;
        }
    }

    /**
     * Sets the intake motor speed.
     *
     * @param speed Motor speed percentage (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        if (intakeMotor != null) {
            intakeMotor.setControl(dutyCycleRequest.withOutput(speed));
        }
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        if (intakeMotor != null) {
            intakeMotor.setControl(dutyCycleRequest.withOutput(0.0));
        }
    }

    /**
     * Creates a command to run intake at default speed.
     *
     * @return Command that runs intake motor
     */
    public Command intakeCommand() {
        return run(() -> setSpeed(INTAKE_SPEED)).withName("Intake: Run");
    }

    /**
     * Creates a command to run intake at custom speed.
     *
     * @param speed Motor speed percentage (-1.0 to 1.0)
     * @return Command that runs intake at specified speed
     */
    public Command intakeCommand(double speed) {
        return run(() -> setSpeed(speed)).withName("Intake: " + (int)(speed * 100) + "%");
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

