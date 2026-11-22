package com.mercury1089.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The math that turns "the camera sees something at this angle" into "it is here on the field".
 *
 * <p>The idea: a camera sees along a straight line (a ray) leaving its lens. If we know how high
 * the thing is, we can follow the ray until it reaches that height. That point is where the thing
 * is.
 */
public class CameraMath {
  /** Rays closer to level than this are treated as never reaching the target */
  private static final double MIN_RAY_SLOPE = Math.tan(Math.toRadians(0.5));

  private CameraMath() {}

  /**
   * Where a target seen at these angles is, from the robot's center.
   *
   * @param mount where the camera sits on the robot
   * @param targetHeightMeters how high the middle of the target is above the floor
   * @param txDegrees left-right angle in the image, positive = right
   * @param tyDegrees up-down angle in the image, positive = up
   * @return the target's position (x forward, y left, in meters), or null if the ray never reaches
   *     that height (for example, it points above the horizon)
   */
  public static Translation2d anglesToRobotRelative(
      CameraMount mount, double targetHeightMeters, double txDegrees, double tyDegrees) {
    // Step 1: the direction of the ray, as the camera sees it.
    // x is out of the lens, y is to the left, z is up.
    double x = 1.0;
    double y = -Math.tan(Math.toRadians(txDegrees));
    double z = Math.tan(Math.toRadians(tyDegrees));

    // Step 2: tilt the ray down by the camera's tilt.
    // This spins the ray around the left-right (y) axis, like nodding your head down. A ray
    // straight out of the lens (x = 1, z = 0) ends up pointing down by the tilt: its new z is
    // -sin(tilt). y doesn't change, because nodding doesn't move anything left or right.
    double tilt = Math.toRadians(mount.getTiltDownDegrees());
    double tiltedX = x * Math.cos(tilt) + z * Math.sin(tilt);
    double tiltedZ = -x * Math.sin(tilt) + z * Math.cos(tilt);

    // Step 3: follow the ray until it reaches the target's height
    double heightChange = targetHeightMeters - mount.getHeightMeters();
    if (Math.abs(tiltedZ) < 1e-9) {
      return null; // the ray is level, so it never gets there
    }
    double steps = heightChange / tiltedZ;
    boolean behindTheLens = steps <= 0;
    boolean almostLevel = Math.abs(tiltedZ) / Math.hypot(tiltedX, y) < MIN_RAY_SLOPE;
    if (behindTheLens || almostLevel) {
      return null;
    }

    // Step 4: that point, turned the way the camera faces, then moved to the camera's spot
    Translation2d fromLens = new Translation2d(steps * tiltedX, steps * y);
    fromLens = fromLens.rotateBy(Rotation2d.fromDegrees(mount.getFacingDegrees()));
    return new Translation2d(mount.getForwardMeters(), mount.getLeftMeters()).plus(fromLens);
  }

  /**
   * Where a pixel on the floor is, from the robot's center, using the camera's lens calibration.
   * Used for PhotonVision cameras, with the bottom middle of a box, which touches the floor.
   *
   * @param mount where the camera sits on the robot
   * @param fx focal length in pixels, left-right
   * @param fy focal length in pixels, up-down
   * @param cx the image's center pixel, left-right
   * @param cy the image's center pixel, up-down
   * @param pixelX the pixel's column
   * @param pixelY the pixel's row (0 is the top of the image)
   * @return the point on the floor (x forward, y left), or null if the pixel is above the horizon
   */
  public static Translation2d floorPixelToRobotRelative(
      CameraMount mount,
      double fx,
      double fy,
      double cx,
      double cy,
      double pixelX,
      double pixelY) {
    double txDegrees = Math.toDegrees(Math.atan((pixelX - cx) / fx));
    double tyDegrees = Math.toDegrees(Math.atan((cy - pixelY) / fy));
    return anglesToRobotRelative(mount, 0.0, txDegrees, tyDegrees);
  }

  /**
   * PhotonVision measures pitch after turning by the yaw, while Limelight's ty is measured straight
   * in the image. This turns PhotonVision's pitch into a Limelight-style ty.
   */
  public static double photonPitchToTy(double yawDegrees, double pitchDegrees) {
    double tanPitch = Math.tan(Math.toRadians(pitchDegrees));
    double cosYaw = Math.cos(Math.toRadians(yawDegrees));
    return Math.toDegrees(Math.atan(tanPitch / cosYaw));
  }

  /**
   * Turn a position from the robot's center into a field position.
   *
   * @param robotPose where the robot is on the field
   * @param robotRelative the position from the robot's center (x forward, y left)
   * @return the position on the field
   */
  public static Translation2d robotRelativeToField(Pose2d robotPose, Translation2d robotRelative) {
    Translation2d turned = robotRelative.rotateBy(robotPose.getRotation());
    return robotPose.getTranslation().plus(turned);
  }
}
