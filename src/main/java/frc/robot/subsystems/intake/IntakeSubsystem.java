package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.RobotConstants.IntakeConstants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

@Logged
public class IntakeSubsystem extends SubsystemBase {
    SparkMax intakeMotor1;
    SparkMax intakeMotor2;
    SparkMaxConfig intakeMotor1Config;
    SparkMaxConfig intakeMotor2Config;
    static SparkClosedLoopController intakeMotor1Controller;
    private double targetSetpoint = 0; // Sınıf seviyesinde değişken

    public IntakeSubsystem() {
        if (RobotBase.isReal()) {
            intakeMotor1 = new SparkMax(CAN.INTAKE_MOTOR_1, MotorType.kBrushless);
            intakeMotor2 = new SparkMax(CAN.INTAKE_MOTOR_2, MotorType.kBrushless);

            intakeMotor1Controller = intakeMotor1.getClosedLoopController();

            intakeMotor1Config = new SparkMaxConfig();
            intakeMotor2Config = new SparkMaxConfig();

            intakeMotor1Config.closedLoop.maxMotion.allowedClosedLoopError(.75);
            intakeMotor1Config.closedLoop.maxMotion.maxVelocity(IntakeConstants.MAX_MOTOR_RPM);
            intakeMotor1Config.closedLoop.maxMotion.maxAcceleration(IntakeConstants.MAX_MOTOR_ACCELERATION);

            intakeMotor1Config.closedLoop.pid(0.5, 0.0, 2);

            intakeMotor2Config.follow(CAN.INTAKE_MOTOR_1, true);

            intakeMotor1.configure(intakeMotor1Config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
            intakeMotor2.configure(intakeMotor2Config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        }
    }

    // Setpoint değerini limitlere göre sınırlayan yeni metod
    private double limitSetpoint(double setpoint) {
        if (setpoint < IntakeConstants.INTAKE_MAX_HEIGHT) {
            return IntakeConstants.INTAKE_MAX_HEIGHT;
        } else if (setpoint > IntakeConstants.INTAKE_MIN_HEIGHT) {
            return IntakeConstants.INTAKE_MIN_HEIGHT;
        }
        return setpoint;
    }

    public void goToSetpoint(double setpoint) {
        // Setpoint değerini limitlere göre sınırla
        double limitedSetpoint = limitSetpoint(setpoint);
        this.targetSetpoint = limitedSetpoint;

        // Add code here to move the intake to the scoring height
        if (RobotBase.isReal()) {
            intakeMotor1Controller.setReference(limitedSetpoint, ControlType.kMAXMotionPositionControl,
                    ClosedLoopSlot.kSlot0, 0.2);
        }
    }

    public boolean atSetpoint() {
        double currentPosition = getEncoderValue(); // Intake'in mevcut pozisyonu
        double tolerance = 1.0; // Tolerans değeri
        return Math.abs(currentPosition - targetSetpoint) <= tolerance;
    }

    public void setEncoderValue(double value) {
        // In rotations
        if (intakeMotor1 != null) {
            intakeMotor1.getEncoder().setPosition(value);
        }
    }

    public double getEncoderValue() {
        if (intakeMotor1 != null) {
            return intakeMotor1.getEncoder().getPosition();
        }
        return 0.0;
    }

    public void setMotorVoltage(double volts) {
        if (intakeMotor1 != null) {
            intakeMotor1.setVoltage(volts);
        }
    }

    public Command goToSetpointCommand(double setpoint) {
        return new InstantCommand(() -> goToSetpoint(setpoint), this);
    }

    /**
     * Motoru belirtilen tur sayısı kadar hareket ettirir (mevcut pozisyondan relatif)
     * @param rotations Hareket edilecek tur sayısı (pozitif = yukarı, negatif = aşağı)
     */
    public Command moveRelativeCommand(double rotations) {
        return new InstantCommand(() -> {
            double currentPos = getEncoderValue();
            double targetPos = currentPos + rotations;
            goToSetpoint(targetPos);
        }, this);
    }


    public void moveAtSpeed(double speed) {
        if (intakeMotor1 == null) return;

        // Encoder değeri limitlere ulaştıysa o yönde hareketi durdur
        double currentPosition = intakeMotor1.getEncoder().getPosition();

        if ((currentPosition <= IntakeConstants.INTAKE_MAX_HEIGHT && speed < 0) ||
                (currentPosition >= IntakeConstants.INTAKE_MIN_HEIGHT && speed > 0)) {
            intakeMotor1.set(0);
        } else {
            intakeMotor1.set(speed);
        }
    }

    // public Command homeElevator() {
    // return this.run(() -> intakeMotor1.setVoltage(1)).until(() ->
    // getCurrentDraw() > 30.0)
    // .finallyDo(() -> setEncoderValue(0));
    // }

    public double getCurrentDraw() {
        if (intakeMotor1 != null) {
            return intakeMotor1.getOutputCurrent();
        }
        return 0.0;
    }

    public RelativeEncoder getEncoder() {
        if (intakeMotor1 != null) {
            return intakeMotor1.getEncoder();
        }
        return null;
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            SmartDashboard.putNumber("intake encoder pos", intakeMotor1.getEncoder().getPosition());
            SmartDashboard.putNumber("intake target pos", targetSetpoint);
            SmartDashboard.putBoolean("Intake At Max Height",
                    intakeMotor1.getEncoder().getPosition() <= IntakeConstants.INTAKE_MAX_HEIGHT);
            SmartDashboard.putBoolean("Intake At Min Height",
                    intakeMotor1.getEncoder().getPosition() >= IntakeConstants.INTAKE_MIN_HEIGHT);

            // Sürekli olarak hedef değerini kontrol et ve gerekirse tekrar gönder
            double currentPos = intakeMotor1.getEncoder().getPosition();
            if (Math.abs(currentPos - targetSetpoint) > 0.5) {
                intakeMotor1Controller.setReference(targetSetpoint, ControlType.kMAXMotionPositionControl,
                        ClosedLoopSlot.kSlot0, 0);
            }

            // Eğer encoder değeri limitlerin ötesindeyse, motoru durdur
            if (currentPos < IntakeConstants.INTAKE_MAX_HEIGHT
                    || currentPos > IntakeConstants.INTAKE_MIN_HEIGHT) {
                intakeMotor1.set(0);
            }
        }
    }
}