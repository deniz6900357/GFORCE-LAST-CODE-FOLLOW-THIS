package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    private static final int MOTOR_LEFT_ID = 41;
    private static final int MOTOR_RIGHT_ID = 42; // inverted

    private final SparkMax motorLeft;
    private final SparkMax motorRight;

    public Climb() {
        motorLeft = new SparkMax(MOTOR_LEFT_ID, MotorType.kBrushless);
        motorRight = new SparkMax(MOTOR_RIGHT_ID, MotorType.kBrushless);

        SparkMaxConfig configLeft = new SparkMaxConfig();
        configLeft.idleMode(IdleMode.kBrake);
        configLeft.smartCurrentLimit(80);
        configLeft.voltageCompensation(12.0);
        configLeft.openLoopRampRate(0.0);

        SparkMaxConfig configRight = new SparkMaxConfig();
        configRight.idleMode(IdleMode.kBrake);
        configRight.inverted(true);
        configRight.smartCurrentLimit(80);
        configRight.voltageCompensation(12.0);
        configRight.openLoopRampRate(0.0);

        motorLeft.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorRight.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        motorLeft.set(speed);
        motorRight.set(speed);
    }

    public void stop() {
        motorLeft.set(0);
        motorRight.set(0);
    }
}
