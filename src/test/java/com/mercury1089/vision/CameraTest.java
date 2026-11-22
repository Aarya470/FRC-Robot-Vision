package com.mercury1089.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class CameraTest {
  private static final double CLOSE = 1e-6;

  /** A camera whose clock and pictures come from the test, so no robot is needed */
  private static class TestCamera extends Camera {
    double time = 10.0;
    CameraFrame nextFrame = null;

    @Override
    protected CameraFrame readFrame() {
      CameraFrame frame = nextFrame;
      nextFrame = null;
      return frame;
    }

    @Override
    protected double now() {
      return time;
    }

    void show(CameraTarget... targets) {
      CameraFrame frame = new CameraFrame(time);
      for (CameraTarget target : targets) {
        frame.addAngleTarget(target);
      }
      nextFrame = frame;
      time += 0.02;
    }
  }

  /** A camera 1 m up, tilted 45 degrees down: the middle of the image is 1 m ahead on the floor */
  private static TestCamera mountedCamera() {
    TestCamera camera = new TestCamera();
    camera.mountedAt(0, 0, 1.0, 45, 0);
    return camera;
  }

  @Test
  public void withNoClassesSetEverythingCounts() {
    TestCamera camera = mountedCamera();
    camera.gamePieceHeight(0);
    camera.show(new CameraTarget(7, 0, 0));
    assertTrue(camera.seesSomething());
    assertFalse(camera.seesRobot());
    assertEquals(ThingType.OTHER, camera.getClosest().getType());
    assertEquals(1.0, camera.getClosest().getDistanceMeters(), CLOSE);
  }

  @Test
  public void classesSortRobotsAndGamePieces() {
    TestCamera camera = mountedCamera();
    camera.redRobotClasses(1).blueRobotClasses(2).gamePieceClasses(0);
    camera.show(new CameraTarget(1, 0, 0), new CameraTarget(2, 10, 0), new CameraTarget(0, -5, 0),
        new CameraTarget(9, 0, 0));
    assertEquals(2, camera.getRobots().size());
    assertEquals(1, camera.getGamePieces().size());
    assertEquals(TeamColor.RED, camera.getRobots().get(0).getColor());
    assertEquals(TeamColor.BLUE, camera.getRobots().get(1).getColor());
    assertTrue(camera.seesRobot());
    assertTrue(camera.seesGamePiece());
  }

  @Test
  public void robotsAndPiecesUseTheirOwnHeights() {
    TestCamera camera = mountedCamera();
    camera.robotClasses(1).gamePieceClasses(0).robotHeight(0.5).gamePieceHeight(0.0);
    camera.show(new CameraTarget(1, 0, 0), new CameraTarget(0, 0, 0));
    assertEquals(0.5, camera.getClosestRobot().getDistanceMeters(), CLOSE);
    assertEquals(1.0, camera.getClosestGamePiece().getDistanceMeters(), CLOSE);
  }

  @Test
  public void withinChecks() {
    TestCamera camera = mountedCamera();
    camera.robotClasses(1).robotHeight(0.0);
    camera.show(new CameraTarget(1, 0, 0));
    assertTrue(camera.seesRobotWithin(1.5));
    assertFalse(camera.seesRobotWithin(0.5));
    assertFalse(camera.seesGamePieceWithin(5));
  }

  @Test
  public void closestIsNullWhenNothingIsSeen() {
    TestCamera camera = mountedCamera();
    camera.show();
    assertNull(camera.getClosest());
    assertNull(camera.getClosestRobot());
    assertEquals("Sees nothing", camera.getStatus());
  }

  @Test
  public void oldSightingsAreForgotten() {
    TestCamera camera = mountedCamera();
    camera.show(new CameraTarget(0, 0, 0));
    assertTrue(camera.seesSomething());
    camera.time += 0.3; // no new frame for longer than maxAge
    assertFalse(camera.seesSomething());
  }

  @Test
  public void fieldPositionNeedsThePose() {
    TestCamera camera = mountedCamera();
    camera.gamePieceHeight(0);
    camera.show(new CameraTarget(0, 0, 0));
    assertNull(camera.getClosest().getFieldPosition());

    TestCamera withPose = mountedCamera();
    withPose.gamePieceHeight(0).usePose(() -> new Pose2d(5, 3, Rotation2d.fromDegrees(90)));
    withPose.show(new CameraTarget(0, 0, 0));
    Translation2d field = withPose.getClosest().getFieldPosition();
    assertNotNull(field);
    assertEquals(5.0, field.getX(), CLOSE);
    assertEquals(4.0, field.getY(), CLOSE);
  }

  @Test
  public void statusExplainsProblems() {
    TestCamera notMounted = new TestCamera();
    notMounted.show(new CameraTarget(0, 0, 0));
    assertEquals("Not mounted: call mountedAt(...)", notMounted.getStatus());

    TestCamera wrongClass = mountedCamera();
    wrongClass.gamePieceClasses(0);
    wrongClass.show(new CameraTarget(4, 0, 0));
    assertEquals("Ignored class 4", wrongClass.getStatus());

    TestCamera tooFar = mountedCamera();
    tooFar.maxDistance(0.5).gamePieceHeight(0);
    tooFar.show(new CameraTarget(0, 0, 0));
    assertEquals("Farther than maxDistance", tooFar.getStatus());

    TestCamera level = new TestCamera();
    level.mountedAt(0, 0, 0.4, 10, 0).robotClasses(1);
    level.show(new CameraTarget(1, 0, 0));
    assertTrue(level.getStatus().startsWith("Camera is level"));

    TestCamera ok = mountedCamera();
    ok.gamePieceClasses(0);
    ok.show(new CameraTarget(0, 0, 0));
    assertEquals("OK", ok.getStatus());
  }

  @Test
  public void fieldPositionsCameraTurnsFieldIntoRobotRelative() {
    List<Translation2d> seen = new ArrayList<>();
    seen.add(new Translation2d(5, 4));
    Camera camera =
        new FieldPositionsCamera(ThingType.ROBOT, () -> seen) {
          @Override
          protected double now() {
            return 1.0;
          }
        };
    assertEquals("Needs the robot's pose: call usePose(...)", camera.getStatus());

    Camera withPose =
        new FieldPositionsCamera(ThingType.ROBOT, () -> seen) {
          @Override
          protected double now() {
            return 1.0;
          }
        }.usePose(() -> new Pose2d(5, 3, Rotation2d.fromDegrees(90)));
    Sighting robot = withPose.getClosestRobot();
    assertNotNull(robot);
    assertEquals(1.0, robot.getRobotRelativePosition().getX(), CLOSE); // straight ahead
    assertEquals(0.0, robot.getRobotRelativePosition().getY(), CLOSE);
    assertEquals(0.0, robot.getAngleDegrees(), CLOSE);
  }

  @Test
  public void limelightNumbersAreRead() {
    double[] raw = {
      1, 4.5, -3.0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0,
      2, -10.0, 1.5, 0.1, 0, 0, 0, 0, 0, 0, 0, 0,
      7, 1, 2 // a cut-off detection is ignored
    };
    List<CameraTarget> targets = LimelightCamera.parse(raw);
    assertEquals(2, targets.size());
    assertEquals(1, targets.get(0).getClassId());
    assertEquals(4.5, targets.get(0).getTxDegrees(), CLOSE);
    assertEquals(-3.0, targets.get(0).getTyDegrees(), CLOSE);
    assertEquals(2, targets.get(1).getClassId());
    assertTrue(LimelightCamera.parse(new double[0]).isEmpty());
    assertTrue(LimelightCamera.parse(null).isEmpty());
  }

  @Test
  public void statusSaysWhenNoClassesAreSet() {
    TestCamera camera = mountedCamera();
    assertFalse(camera.hasClassesSet());
    camera.show(new CameraTarget(7, 0, 0));
    assertEquals("OK (no classes set: everything is OTHER)", camera.getStatus());

    TestCamera withClasses = mountedCamera();
    withClasses.gamePieceClasses(7);
    assertTrue(withClasses.hasClassesSet());
    withClasses.show(new CameraTarget(7, 0, 0));
    assertEquals("OK", withClasses.getStatus());
  }

  @Test
  public void fieldPositionCamerasDontNeedClasses() {
    assertTrue(Camera.fromFieldPositions(ThingType.ROBOT, () -> List.of()).hasClassesSet());
  }

  @Test
  public void statusStopsSayingOkOnceSightingsAreTooOld() {
    TestCamera camera = mountedCamera();
    camera.gamePieceClasses(7);
    camera.show(new CameraTarget(7, 0, 0));
    assertEquals("OK", camera.getStatus());
    camera.time += 1.0; // no new frames for a second
    assertEquals("Sees nothing", camera.getStatus());
  }

  @Test
  public void placesThingsWithThePoseFromWhenTheFrameWasTaken() {
    TestCamera camera = mountedCamera();
    camera.gamePieceClasses(7).gamePieceHeight(0);
    Pose2d[] robot = {new Pose2d(0, 0, Rotation2d.kZero)};
    camera.usePose(() -> robot[0]);

    double takenAt = camera.time;
    camera.getStatus(); // the camera notes where the robot is now
    camera.time += 0.1;
    robot[0] = new Pose2d(2, 0, Rotation2d.kZero); // the robot drove 2 m before the frame arrived

    CameraFrame late = new CameraFrame(takenAt);
    late.addAngleTarget(new CameraTarget(7, 0, 0)); // 1 m ahead of the camera when it was taken
    camera.nextFrame = late;
    assertEquals(1.0, camera.getClosest().getFieldPosition().getX(), CLOSE);
  }
}
