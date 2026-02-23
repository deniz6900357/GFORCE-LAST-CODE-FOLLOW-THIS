// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

  private static final LoggedTunableNumber torqueCurrentControlTolerance =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlTolerance", 20.0);
  private static final LoggedTunableNumber torqueCurrentControlDebounceTime =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlDebounce", 0.025);
  private static final LoggedTunableNumber atGoalDebounceTime =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);

  private Debouncer torqueCurrentDebouncer =
      new Debouncer(torqueCurrentControlDebounceTime.get(), DebounceType.kFalling);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounceTime.get(), DebounceType.kFalling);
  private boolean lastTorqueCurrentControl = false;
  private long launchCount = 0;

  private boolean atGoal = false;

  public Flywheel(String name, FlywheelIO io) {
    this.name = name;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Update debouncers if tunable numbers changed
    if (torqueCurrentControlDebounceTime.hasChanged(hashCode())) {
      torqueCurrentDebouncer =
          new Debouncer(torqueCurrentControlDebounceTime.get(), DebounceType.kFalling);
    }
    if (atGoalDebounceTime.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounceTime.get(), DebounceType.kFalling);
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

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRadsPerSec) {
    boolean inTolerance =
        Math.abs(inputs.velocityRadsPerSec - velocityRadsPerSec)
            <= torqueCurrentControlTolerance.get();
    boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
    atGoal = atGoalDebouncer.calculate(inTolerance);

    if (!torqueCurrentControl && lastTorqueCurrentControl) {
      launchCount++;
    }
    lastTorqueCurrentControl = torqueCurrentControl;

    outputs.mode =
        torqueCurrentControl
            ? FlywheelIOOutputMode.TORQUE_CURRENT_BANG_BANG
            : FlywheelIOOutputMode.DUTY_CYCLE_BANG_BANG;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    Logger.recordOutput(name + "/Setpoint", velocityRadsPerSec);
  }

  /** Stops the flywheel. */
  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
    atGoal = false;
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

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  // Helper methods for backward compatibility
  public void setVelocity(double velocityRadsPerSec) {
    runVelocity(velocityRadsPerSec);
  }

  public Command runVelocityCommand(double velocityRadsPerSec) {
    return runEnd(() -> runVelocity(velocityRadsPerSec), this::stop)
        .withName("Flywheel: " + Math.round(velocityRadsPerSec) + " rad/s");
  }
}
