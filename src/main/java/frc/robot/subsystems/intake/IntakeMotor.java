package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.CAN;

@Logged
public class IntakeMotor extends SubsystemBase {
    private final SparkMax intakeMotor;
    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

    public IntakeMotor() {
        if (RobotBase.isReal()) {
            intakeMotor = new SparkMax(CAN.INTAKE_SAKSO_MOTOR, MotorType.kBrushless);

            intakeMotorConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
            intakeMotorConfig.smartCurrentLimit(40);

            // Apply configuration
            intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            intakeMotor = null;
        }
    }

    public void setSpeed(double speed) {
        if (intakeMotor != null) {
            intakeMotor.set(speed);
        }
    }

    public void stop() {
        if (intakeMotor != null) {
            intakeMotor.stopMotor();
        }
    }

    public Command intakeInCommand(double speed) {
        double intakeSpeed = Math.abs(speed);
        return new StartEndCommand(() -> setSpeed(intakeSpeed), this::stop, this);
    }

    public Command intakeOutCommand(double speed) {
        double outSpeed = -Math.abs(speed);
        return new StartEndCommand(() -> setSpeed(outSpeed), this::stop, this);
    }
}
