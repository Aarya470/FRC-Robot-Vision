package com.mercury1089.vision;

import edu.wpi.first.math.geometry.Translation2d;

/** Something a camera saw, with where it is. */
public class Sighting {
  private final ThingType type;
  private final TeamColor color;
  private final int classId;
  private final Translation2d robotRelativePosition;
  private final Translation2d fieldPosition;
  private final double timestampSeconds;

  public Sighting(
      ThingType type,
      TeamColor color,
      int classId,
      Translation2d robotRelativePosition,
      Translation2d fieldPosition,
      double timestampSeconds) {
    this.type = type;
    this.color = color;
    this.classId = classId;
    this.robotRelativePosition = robotRelativePosition;
    this.fieldPosition = fieldPosition;
    this.timestampSeconds = timestampSeconds;
  }

  /** @return ROBOT, GAME_PIECE or OTHER */
  public ThingType getType() {
    return type;
  }

  /** @return true if this is a robot */
  public boolean isRobot() {
    return type == ThingType.ROBOT;
  }

  /** @return true if this is a game piece */
  public boolean isGamePiece() {
    return type == ThingType.GAME_PIECE;
  }

  /** @return RED or BLUE for robots whose classes you set, otherwise UNKNOWN */
  public TeamColor getColor() {
    return color;
  }

  /** @return the class number the detection model gave it, or -1 if the camera doesn't say */
  public int getClassId() {
    return classId;
  }

  /**
   * Where it is from the robot's center, in meters. X is forward (negative = behind the robot)
   * and Y is left (negative = to the right).
   */
  public Translation2d getRobotRelativePosition() {
    return robotRelativePosition;
  }

  /**
   * Where it is on the field, in meters, like your robot's pose. This is null if the camera was
   * not given the robot's pose with {@link Camera#usePose}.
   */
  public Translation2d getFieldPosition() {
    return fieldPosition;
  }

  /** @return how far it is from the robot's center, in meters */
  public double getDistanceMeters() {
    return robotRelativePosition.getNorm();
  }

  /**
   * @return which direction it is from the robot, in degrees. 0 is straight ahead, 90 is to the
   *     left, -90 is to the right, and 180 is behind.
   */
  public double getAngleDegrees() {
    return Math.toDegrees(Math.atan2(robotRelativePosition.getY(), robotRelativePosition.getX()));
  }

  /** @return when the camera saw it, in seconds (the same clock as Timer.getFPGATimestamp()) */
  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  @Override
  public String toString() {
    return type
        + " "
        + (color == TeamColor.UNKNOWN ? "" : color + " ")
        + String.format("%.2f m away at %.0f degrees", getDistanceMeters(), getAngleDegrees());
  }
}
