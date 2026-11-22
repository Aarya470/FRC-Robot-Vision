package com.mercury1089.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Supplier;

/** Any system that gives field positions. Made with {@link Camera#fromFieldPositions}. */
class FieldPositionsCamera extends Camera {
  private final ThingType type;
  private final Supplier<List<Translation2d>> positions;

  FieldPositionsCamera(ThingType type, Supplier<List<Translation2d>> positions) {
    this.type = type;
    this.positions = positions;
  }

  /** This camera is told what it sees, so it needs no class numbers. */
  @Override
  public boolean hasClassesSet() {
    return true;
  }

  @Override
  protected CameraFrame readFrame() {
    Pose2d robot = getRobotPoseOrNull();
    if (robot == null) {
      return CameraFrame.problem(now(), "Needs the robot's pose: call usePose(...)");
    }
    CameraFrame frame = new CameraFrame(now());
    List<Translation2d> list = positions.get();
    if (list != null) {
      for (Translation2d fieldPosition : list) {
        // Turn the field position into a position from the robot's center
        Translation2d offset = fieldPosition.minus(robot.getTranslation());
        Translation2d robotRelative = offset.rotateBy(robot.getRotation().unaryMinus());
        frame.addFoundTarget(new FoundTarget(-1, type, robotRelative));
      }
    }
    return frame;
  }
}
