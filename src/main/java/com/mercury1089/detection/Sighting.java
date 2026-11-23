package com.mercury1089.detection;

import edu.wpi.first.math.geometry.Translation2d;

/** One robot or game piece that a camera sees. */
public class Sighting {
  /** What was seen */
  public enum Type {
    ROBOT,
    GAME_PIECE
  }

  /** Which alliance a robot is on, if the camera can tell */
  public enum Color {
    RED,
    BLUE,
    UNKNOWN
  }

  private final Type type;
  private final Color color;
  private final int classId;
  private final Translation2d robotRelativePosition;
  private final Translation2d fieldPosition;
  private final double timestampSeconds;

  Sighting(
      Type type,
      Color color,
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

  /** @return ROBOT or GAME_PIECE */
  public Type getType() {
    return type;
  }

  /** @return true if this is a robot */
  public boolean isRobot() {
    return type == Type.ROBOT;
  }

  /** @return true if this is a game piece */
  public boolean isGamePiece() {
    return type == Type.GAME_PIECE;
  }

  /** @return RED or BLUE for robots seen with redRobotClasses or blueRobotClasses, else UNKNOWN */
  public Color getColor() {
    return color;
  }

  /** @return the class number the camera's model gave it */
  public int getClassId() {
    return classId;
  }

  /** @return where it is from the robot's center, in meters (x forward, y left) */
  public Translation2d getRobotRelativePosition() {
    return robotRelativePosition;
  }

  /** @return where it is on the field, in meters, or null if the camera has no usePose(...) */
  public Translation2d getFieldPosition() {
    return fieldPosition;
  }

  /** @return how far it is from the robot's center, in meters */
  public double getDistanceMeters() {
    return robotRelativePosition.getNorm();
  }

  /** @return which way it is from the robot: 0 = ahead, 90 = left, -90 = right, 180 = behind */
  public double getAngleDegrees() {
    return robotRelativePosition.getAngle().getDegrees();
  }

  /** @return when the camera took the picture, in seconds (FPGA time) */
  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  @Override
  public String toString() {
    return String.format(
        "%s (class %d, %s) %.2f m away", type, classId, color, getDistanceMeters());
  }
}
