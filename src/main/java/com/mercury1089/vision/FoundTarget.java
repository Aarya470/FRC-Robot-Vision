package com.mercury1089.vision;

import edu.wpi.first.math.geometry.Translation2d;

/** A target whose position the camera already worked out, such as with lens calibration. */
public class FoundTarget {
  private final int classId;
  private final ThingType type;
  private final Translation2d robotRelativePosition;

  /**
   * @param classId the model's class number, or -1 if there is none
   * @param type what it is, or null to work it out from the class number
   * @param robotRelativePosition where it is from the robot's center (x forward, y left)
   */
  public FoundTarget(int classId, ThingType type, Translation2d robotRelativePosition) {
    this.classId = classId;
    this.type = type;
    this.robotRelativePosition = robotRelativePosition;
  }

  public int getClassId() {
    return classId;
  }

  public ThingType getType() {
    return type;
  }

  public Translation2d getRobotRelativePosition() {
    return robotRelativePosition;
  }
}
