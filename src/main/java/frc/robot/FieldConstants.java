// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Field constants for the 2026 REBUILT game.
 * All measurements are in meters.
 */
public final class FieldConstants {
  private FieldConstants() {
    throw new AssertionError("Utility class");
  }

  // Field dimensions (2026 REBUILT official field)
  public static final double fieldLength = 16.54;
  public static final double fieldWidth = 8.21;

  /** Hub constants (center of AprilTag clusters) */
  public static final class Hub {
    // Hub center point (blue alliance, meters)
    // Calculated from average of AprilTags 17-28
    // X: 4.552m (center of tag cluster), Y: 4.021m (vertical center), Z: 2.1m (shooter height)
    public static final Translation3d topCenterPoint = new Translation3d(4.552, 4.021, 2.1);
  }
}
