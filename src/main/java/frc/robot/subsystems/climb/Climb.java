package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    private static final int MOTOR_LEFT_ID = 41;
    private static final int MOTOR_RIGHT_ID = 42; // inverted

    private static final double kP = 0.15;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double POSITION_TOLERANCE = 1.0;

    private final SparkMax motorLeft;
    private final SparkMax motorRight;
    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;

    private final PIDController pidLeft;
    private final PIDController pidRight;

    private boolean positionControlEnabled = false;

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

        encoderLeft = motorLeft.getEncoder();
        encoderRight = motorRight.getEncoder();

        pidLeft = new PIDController(kP, kI, kD);
        pidRight = new PIDController(kP, kI, kD);
        pidLeft.setTolerance(POSITION_TOLERANCE);
        pidRight.setTolerance(POSITION_TOLERANCE);
    }

    @Override
    public void periodic() {
        if (positionControlEnabled) {
            double outputLeft = pidLeft.calculate(encoderLeft.getPosition());
            double outputRight = pidRight.calculate(encoderRight.getPosition());
            motorLeft.set(outputLeft);
            motorRight.set(outputRight);
        }

        SmartDashboard.putNumber("Climb/Left Position", encoderLeft.getPosition());
        SmartDashboard.putNumber("Climb/Right Position", encoderRight.getPosition());
        SmartDashboard.putNumber("Climb/Left Velocity", encoderLeft.getVelocity());
        SmartDashboard.putNumber("Climb/Right Velocity", encoderRight.getVelocity());
        SmartDashboard.putNumber("Climb/Left Setpoint", pidLeft.getSetpoint());
        SmartDashboard.putNumber("Climb/Right Setpoint", pidRight.getSetpoint());
    }

    public void setPosition(double target) {
        positionControlEnabled = true;
        pidLeft.setSetpoint(target);
        pidRight.setSetpoint(target);
    }

    public void setSpeed(double speed) {
        positionControlEnabled = false;
        motorLeft.set(speed);
        motorRight.set(speed);
    }

    public void setSpeedIndependent(double leftSpeed, double rightSpeed) {
        positionControlEnabled = false;
        motorLeft.set(leftSpeed);
        motorRight.set(rightSpeed);
    }

    public void stop() {
        positionControlEnabled = false;
        motorLeft.set(0);
        motorRight.set(0);
    }
}
