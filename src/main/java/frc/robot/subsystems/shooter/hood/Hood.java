// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {
  private static final double minAngle = ShooterConstants.Hood.MIN_ANGLE_RAD;
  private static final double maxAngle = ShooterConstants.Hood.MAX_ANGLE_RAD;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  static {
    kP.initDefault(100.0);  // Motion Magic onboard PID (tune as needed)
    kD.initDefault(5.0);    // Motion Magic damping (tune as needed)
    toleranceDeg.initDefault(1.0);
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private boolean motorDisconnectedAlertActive = false;

  private BooleanSupplier coastOverride = () -> false;

  private double goalAngle = 0.0;
  private double goalVelocity = 0.0;

  private static double hoodOffset = 0.0;
  private boolean hoodZeroed = false;

  public Hood(HoodIO io) {
    this.io = io;

    // Auto-zero on startup so robot is immediately ready
    zero();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Motor disconnection alert
    boolean motorDisconnected = !motorConnectedDebouncer.calculate(inputs.motorConnected);
    if (Robot.showHardwareAlerts() && motorDisconnected && !motorDisconnectedAlertActive) {
      System.err.println("WARNING: Hood motor disconnected!");
      motorDisconnectedAlertActive = true;
    } else if (!motorDisconnected) {
      motorDisconnectedAlertActive = false;
    }

    // Stop when disabled
    if (DriverStation.isDisabled() || !hoodZeroed) {
      outputs.mode = HoodIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = HoodIOOutputMode.COAST;
      }
    }

    // Update tunable numbers
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    // Dashboard telemetry - Elastic/Shuffleboard
    Logger.recordOutput("Hood/CurrentAngleDeg", Math.toDegrees(getMeasuredAngleRad()));
    Logger.recordOutput("Hood/GoalAngleDeg", Math.toDegrees(goalAngle));
    Logger.recordOutput("Hood/AtGoal", atGoal());
    Logger.recordOutput("Hood/IsZeroed", hoodZeroed);
    Logger.recordOutput("Hood/AngleErrorDeg", Math.toDegrees(Math.abs(getMeasuredAngleRad() - goalAngle)));
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled() && hoodZeroed) {
      outputs.positionRad = MathUtil.clamp(goalAngle, minAngle, maxAngle) - hoodOffset;
      outputs.velocityRadsPerSec = goalVelocity;
      outputs.mode = HoodIOOutputMode.CLOSED_LOOP;

      // Log state
      Logger.recordOutput("Hood/Profile/GoalPositionRad", goalAngle);
      Logger.recordOutput("Hood/Profile/GoalVelocityRadPerSec", goalVelocity);
    }

    io.applyOutputs(outputs);
  }

  private void setGoalParams(double angle, double velocity) {
    goalAngle = angle;
    goalVelocity = velocity;
  }

  public double getMeasuredAngleRad() {
    return inputs.positionRads + hoodOffset;
  }

  public boolean atGoal() {
    return DriverStation.isEnabled()
        && hoodZeroed
        && Math.abs(getMeasuredAngleRad() - goalAngle)
            <= Units.degreesToRadians(toleranceDeg.get());
  }

  public boolean isZeroed() {
    return hoodZeroed;
  }

  private void zero() {
    hoodOffset = minAngle - inputs.positionRads;
    hoodZeroed = true;
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    this.coastOverride = coastOverride;
  }

  public void setAngle(double angleRad) {
    setGoalParams(angleRad, 0.0);
  }

  public Command setAngleCommand(double angleRad) {
    return run(() -> setAngle(angleRad))
        .withName("Hood: " + Math.round(Math.toDegrees(angleRad)) + "°");
  }

  public Command stowCommand() {
    return setAngleCommand(minAngle);
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          LaunchCalculator.getInstance().clearLaunchingParameters();
          var params = LaunchCalculator.getInstance().getParameters();
          setGoalParams(params.hoodAngle(), params.hoodVelocity());
        });
  }

  public Command runFixedCommand(DoubleSupplier angle, DoubleSupplier velocity) {
    return run(() -> setGoalParams(angle.getAsDouble(), velocity.getAsDouble()));
  }

  public Command zeroCommand() {
    return runOnce(this::zero).ignoringDisable(true);
  }
}
