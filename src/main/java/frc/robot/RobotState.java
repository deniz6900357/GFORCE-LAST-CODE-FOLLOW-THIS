// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Singleton class that tracks the robot's state (pose, velocity, etc.).
 * This is a simplified version for basic shooter calculations.
 */
public class RobotState {
  private static RobotState instance;

  private Pose2d estimatedPose = new Pose2d();
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();
  private ChassisSpeeds fieldVelocity = new ChassisSpeeds();

  private RobotState() {}

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  /**
   * Updates the robot's estimated pose.
   * Should be called from your drivetrain subsystem's periodic method.
   */
  public void updatePose(Pose2d pose) {
    this.estimatedPose = pose;
  }

  /**
   * Updates the robot's velocity (robot-relative).
   * Should be called from your drivetrain subsystem's periodic method.
   */
  public void updateRobotVelocity(ChassisSpeeds velocity) {
    this.robotVelocity = velocity;
    // Convert to field-relative velocity
    this.fieldVelocity =
        new ChassisSpeeds(
            velocity.vxMetersPerSecond * estimatedPose.getRotation().getCos()
                - velocity.vyMetersPerSecond * estimatedPose.getRotation().getSin(),
            velocity.vxMetersPerSecond * estimatedPose.getRotation().getSin()
                + velocity.vyMetersPerSecond * estimatedPose.getRotation().getCos(),
            velocity.omegaRadiansPerSecond);
  }

  /** Gets the robot's estimated pose. */
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  /** Gets the robot's rotation. */
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  /** Gets the robot-relative velocity. */
  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  /** Gets the field-relative velocity. */
  public ChassisSpeeds getFieldVelocity() {
    return fieldVelocity;
  }
}
