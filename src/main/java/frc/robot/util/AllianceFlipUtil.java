// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

/**
 * Utility for flipping field positions between red and blue alliance.
 */
public class AllianceFlipUtil {

  /**
   * Flips a translation to the correct side of the field based on the current alliance color.
   * Blue alliance uses the translation as-is; red alliance flips it horizontally.
   */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(
          FieldConstants.fieldLength - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance color.
   */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          FieldConstants.fieldLength - pose.getX(),
          pose.getY(),
          new Rotation2d(Math.PI).minus(pose.getRotation()));
    } else {
      return pose;
    }
  }

  /**
   * Returns true if we're on red alliance and need to flip coordinates.
   */
  private static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
}
