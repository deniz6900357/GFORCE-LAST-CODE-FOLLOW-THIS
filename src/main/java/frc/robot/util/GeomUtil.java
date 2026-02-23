// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Geometry utility methods.
 * Used with Lombok's @ExtensionMethod to add methods to geometry classes.
 */
public class GeomUtil {

  /** Converts a Translation3d to a Translation2d by dropping the Z coordinate. */
  public static Translation2d toTranslation2d(Translation3d translation3d) {
    return new Translation2d(translation3d.getX(), translation3d.getY());
  }

  /** Converts a Transform3d to a Transform2d by dropping the Z component. */
  public static Transform2d toTransform2d(Transform3d transform3d) {
    return new Transform2d(
        toTranslation2d(transform3d.getTranslation()), transform3d.getRotation().toRotation2d());
  }

  /** Converts a Pose3d to a Pose2d by dropping the Z component. */
  public static Pose2d toPose2d(Pose3d pose3d) {
    return new Pose2d(toTranslation2d(pose3d.getTranslation()), pose3d.getRotation().toRotation2d());
  }
}
