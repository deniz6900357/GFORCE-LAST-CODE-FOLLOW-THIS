// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  private final String name;
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollowerConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private boolean motorDisconnectedAlertActive = false;
  private boolean followerDisconnectedAlertActive = false;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.4);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.22);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.019);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Flywheel/MaxAcceleration", 250.0);

  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(250.0);

  private boolean atGoal = false;

  public Flywheel(String name, FlywheelIO io) {
    this.name = name;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // PID gains'i outputs'a aktar
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    // Slew rate limiter güncelle
    if (maxAcceleration.hasChanged(hashCode())) {
      slewRateLimiter = new SlewRateLimiter(maxAcceleration.get());
    }

    // Motor disconnection alerts
    boolean motorDisconnected = !motorConnectedDebouncer.calculate(inputs.connected);
    boolean followerDisconnected =
        !motorFollowerConnectedDebouncer.calculate(inputs.followerConnected);

    if (Robot.showHardwareAlerts() && motorDisconnected && !motorDisconnectedAlertActive) {
      System.err.println("WARNING: Flywheel motor disconnected!");
      motorDisconnectedAlertActive = true;
    } else if (!motorDisconnected) {
      motorDisconnectedAlertActive = false;
    }

    if (Robot.showHardwareAlerts() && followerDisconnected && !followerDisconnectedAlertActive) {
      System.err.println("WARNING: Flywheel follower motor disconnected!");
      followerDisconnectedAlertActive = true;
    } else if (!followerDisconnected) {
      followerDisconnectedAlertActive = false;
    }
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput(name + "/Mode", outputs.mode);
    io.applyOutputs(outputs);
  }

  /** Run closed loop at the specified velocity with slew rate limiting and feedforward. */
  private void runVelocity(double velocityRadsPerSec) {
    double setpointRadPerSec = slewRateLimiter.calculate(velocityRadsPerSec);
    atGoal = Math.abs(setpointRadPerSec - velocityRadsPerSec) < 2.0;

    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = setpointRadPerSec;
    outputs.feedforward =
        Math.signum(setpointRadPerSec) * kS.get() + setpointRadPerSec * kV.get();

    Logger.recordOutput(name + "/Setpoint", setpointRadPerSec);
    Logger.recordOutput(name + "/Goal", velocityRadsPerSec);
  }

  /** Stops the flywheel. */
  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
    atGoal = false;
    slewRateLimiter.reset(inputs.velocityRadsPerSec);
  }

  /** Returns the current velocity in rad/s. */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public boolean atGoal() {
    return atGoal;
  }

  public Command runTrackTargetCommand() {
    return runEnd(
        () -> runVelocity(LaunchCalculator.getInstance().getParameters().flywheelSpeed()),
        this::stop);
  }

  public Command runFixedCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command runVelocityCommand(double velocityRadsPerSec) {
    return runEnd(() -> runVelocity(velocityRadsPerSec), this::stop)
        .withName("Flywheel: " + Math.round(velocityRadsPerSec) + " rad/s");
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public void setVelocity(double velocityRadsPerSec) {
    runVelocity(velocityRadsPerSec);
  }
}
