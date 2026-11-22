package com.mercury1089.vision;

/**
 * Where a camera sits on the robot. All distances are measured from the robot's center, on the
 * floor.
 */
public class CameraMount {
  private final double forwardMeters;
  private final double leftMeters;
  private final double heightMeters;
  private final double tiltDownDegrees;
  private final double facingDegrees;

  /**
   * @param forwardMeters how far in front of the robot's center (negative = behind)
   * @param leftMeters how far left of the robot's center (negative = right)
   * @param heightMeters how high the lens is above the floor
   * @param tiltDownDegrees how far the camera is tilted down (0 = level)
   * @param facingDegrees which way it points: 0 = forward, 90 = left, 180 = backward, -90 = right
   */
  public CameraMount(
      double forwardMeters,
      double leftMeters,
      double heightMeters,
      double tiltDownDegrees,
      double facingDegrees) {
    this.forwardMeters = forwardMeters;
    this.leftMeters = leftMeters;
    this.heightMeters = heightMeters;
    this.tiltDownDegrees = tiltDownDegrees;
    this.facingDegrees = facingDegrees;
  }

  public double getForwardMeters() {
    return forwardMeters;
  }

  public double getLeftMeters() {
    return leftMeters;
  }

  public double getHeightMeters() {
    return heightMeters;
  }

  public double getTiltDownDegrees() {
    return tiltDownDegrees;
  }

  public double getFacingDegrees() {
    return facingDegrees;
  }
}
