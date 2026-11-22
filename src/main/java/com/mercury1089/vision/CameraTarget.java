package com.mercury1089.vision;

/**
 * One thing a camera sees, given as angles from the middle of the image. This is what a
 * Limelight's tx and ty are, and what most other cameras can give you too.
 */
public class CameraTarget {
  private final int classId;
  private final double txDegrees;
  private final double tyDegrees;

  /**
   * @param classId the class number the detection model gave it (for example 0 = game piece)
   * @param txDegrees left-right angle to the middle of the box. Positive means right of center.
   * @param tyDegrees up-down angle to the middle of the box. Positive means above center.
   */
  public CameraTarget(int classId, double txDegrees, double tyDegrees) {
    this.classId = classId;
    this.txDegrees = txDegrees;
    this.tyDegrees = tyDegrees;
  }

  public int getClassId() {
    return classId;
  }

  public double getTxDegrees() {
    return txDegrees;
  }

  public double getTyDegrees() {
    return tyDegrees;
  }

  @Override
  public String toString() {
    return "CameraTarget(class " + classId + ", tx " + txDegrees + ", ty " + tyDegrees + ")";
  }
}
