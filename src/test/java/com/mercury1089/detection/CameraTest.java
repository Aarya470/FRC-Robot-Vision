package com.mercury1089.detection;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
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

  /** A camera whose pictures the test makes by hand, with a clock the test moves */
  static class TestCamera extends Camera {
    double time = 100.0;
    Frame next = null;

    /** The next picture shows these things (class number, tx, ty) */
    void show(double[]... things) {
      next = new Frame(time);
      for (double[] thing : things) {
        next.detections.add(new Detection((int) thing[0], thing[1], thing[2]));
      }
      time += 0.02;
    }

    @Override
    Frame readFrame() {
      Frame frame = next;
      next = null;
      return frame;
    }

    @Override
    protected double now() {
      return time;
    }
  }

  /** 1 m up, tilted 45 degrees down: something on the floor in the middle is 1 m ahead */
  static TestCamera camera() {
    TestCamera camera = new TestCamera();
    camera.mountedAt(0, 0, 1.0, 45, 0).gamePieceHeight(0).robotHeight(0).gamePieceClasses(0);
    camera.redRobotClasses(1).blueRobotClasses(2);
    return camera;
  }

  // ---------------------------------------------------------------- the math

  @Test
  public void straightAheadOnTheFloor() {
    Translation2d p = camera().anglesToPosition(0, 0, 0);
    assertEquals(1.0, p.getX(), CLOSE);
    assertEquals(0.0, p.getY(), CLOSE);
  }

  @Test
  public void higherInThePictureIsFarther() {
    // 30 down plus 10 up in the picture = 20 degrees below level
    TestCamera camera = new TestCamera();
    camera.mountedAt(0, 0, 1.0, 30, 0);
    assertEquals(1.0 / Math.tan(Math.toRadians(20)), camera.anglesToPosition(0, 10, 0).getX(), CLOSE);
  }

  @Test
  public void positiveTxIsToTheRight() {
    Translation2d p = camera().anglesToPosition(20, 0, 0);
    assertTrue(p.getY() < 0);
    assertTrue(p.getX() > 0);
  }

  @Test
  public void aBackwardCameraBehindTheCenter() {
    TestCamera camera = new TestCamera();
    camera.mountedAt(-0.3, 0, 1.0, 45, 180);
    Translation2d p = camera.anglesToPosition(0, 0, 0);
    assertEquals(-1.3, p.getX(), CLOSE);
    assertEquals(0.0, p.getY(), CLOSE);
  }

  @Test
  public void aTallerThingIsCloser() {
    assertEquals(0.5, camera().anglesToPosition(0, 0, 0.5).getX(), CLOSE);
  }

  @Test
  public void aLowCameraLooksUpAtARobot() {
    TestCamera camera = new TestCamera();
    camera.mountedAt(0, 0, 0.13, 0, 0);
    double ty = Math.toDegrees(Math.atan(0.27 / 2.0));
    assertEquals(2.0, camera.anglesToPosition(0, ty, 0.4).getX(), CLOSE);
  }

  @Test
  public void aboveTheHorizonCantBePlaced() {
    TestCamera camera = new TestCamera();
    camera.mountedAt(0, 0, 1.0, 0, 0);
    assertNull(camera.anglesToPosition(0, 5, 0));
  }

  @Test
  public void photonPitchTurnsIntoTy() {
    // Something 3 m ahead and 1.2 m right, 0.35 m below a level camera
    double ny = -0.35 / 3.0;
    double yaw = Math.toDegrees(Math.atan(1.2 / 3.0));
    double photonPitch = Math.toDegrees(Math.atan(ny * Math.cos(Math.toRadians(yaw))));
    assertEquals(Math.toDegrees(Math.atan(ny)), PhotonVisionCamera.pitchToTy(yaw, photonPitch), 1e-9);
  }

  @Test
  public void readsLimelightNumbers() {
    double[] raw = new double[24];
    raw[0] = 3;
    raw[1] = -5;
    raw[2] = 2;
    raw[12] = 1;
    raw[13] = 7;
    raw[14] = -4;
    Camera.Frame frame = new Camera.Frame(0);
    LimelightCamera.readDetections(raw, frame);
    assertEquals(2, frame.detections.size());
    assertEquals(3, frame.detections.get(0).classId);
    assertEquals(7, frame.detections.get(1).txDegrees, CLOSE);
    assertEquals(-4, frame.detections.get(1).tyDegrees, CLOSE);
  }

  // ---------------------------------------------------------------- what it sees

  @Test
  public void sortsRobotsAndGamePieces() {
    TestCamera camera = camera();
    camera.show(new double[] {0, 0, 0}, new double[] {1, 10, 0}, new double[] {2, -10, 0});
    assertTrue(camera.seesSomething());
    assertTrue(camera.seesGamePiece());
    assertEquals(2, camera.getRobots().size());
    assertEquals(1, camera.getGamePieces().size());
    assertEquals("OK", camera.getStatus());

    List<Sighting.Color> colors = new ArrayList<>();
    for (Sighting robot : camera.getRobots()) {
      colors.add(robot.getColor());
    }
    assertTrue(colors.contains(Sighting.Color.RED));
    assertTrue(colors.contains(Sighting.Color.BLUE));
  }

  @Test
  public void closestAndWithin() {
    TestCamera camera = camera();
    camera.show(new double[] {0, 0, 0}, new double[] {0, 0, -20});
    assertEquals(camera.anglesToPosition(0, -20, 0).getNorm(), camera.getClosestGamePiece().getDistanceMeters(), CLOSE);
    assertTrue(camera.seesGamePieceWithin(1.0));
    assertFalse(camera.seesRobot());
    assertNull(camera.getClosestRobot());
  }

  @Test
  public void ignoresClassesItWasntToldAbout() {
    TestCamera camera = camera();
    camera.show(new double[] {9, 0, 0});
    assertFalse(camera.seesSomething());
    assertEquals("Ignored class 9", camera.getStatus());
  }

  @Test
  public void needsClassesAndAMount() {
    TestCamera noClasses = new TestCamera();
    noClasses.mountedAt(0, 0, 1, 45, 0);
    assertFalse(noClasses.hasClassesSet());
    noClasses.show(new double[] {0, 0, 0});
    assertFalse(noClasses.seesSomething());
    assertTrue(noClasses.getStatus().startsWith("No classes set"));

    TestCamera notMounted = new TestCamera();
    notMounted.gamePieceClasses(0);
    notMounted.show(new double[] {0, 0, 0});
    assertEquals("Not mounted: call mountedAt(...)", notMounted.getStatus());
  }

  @Test
  public void ignoresThingsTooFarAway() {
    TestCamera camera = camera();
    camera.maxDistance(0.5);
    camera.show(new double[] {0, 0, 0});
    assertFalse(camera.seesSomething());
    assertEquals("Farther than maxDistance", camera.getStatus());
  }

  @Test
  public void forgetsOldSightings() {
    TestCamera camera = camera();
    camera.show(new double[] {0, 0, 0});
    assertTrue(camera.seesGamePiece());
    camera.time += 1.0; // no new pictures for a second
    assertFalse(camera.seesGamePiece());
    assertEquals("Sees nothing", camera.getStatus());
  }

  @Test
  public void givesFieldPositionsWithAPose() {
    TestCamera camera = camera();
    camera.usePose(() -> new Pose2d(5, 3, Rotation2d.fromDegrees(90)));
    camera.show(new double[] {0, 0, 0}); // 1 m ahead
    Translation2d field = camera.getClosestGamePiece().getFieldPosition();
    assertEquals(5.0, field.getX(), CLOSE);
    assertEquals(4.0, field.getY(), CLOSE);
  }

  @Test
  public void usesThePoseFromWhenThePictureWasTaken() {
    TestCamera camera = camera();
    Pose2d[] robot = {new Pose2d()};
    camera.usePose(() -> robot[0]);

    double takenAt = camera.time;
    camera.getStatus(); // the camera notes where the robot is now
    camera.time += 0.1;
    robot[0] = new Pose2d(2, 0, Rotation2d.kZero); // the robot drove 2 m before the picture arrived

    camera.next = new Camera.Frame(takenAt);
    camera.next.detections.add(new Camera.Detection(0, 0, 0)); // 1 m ahead when it was taken
    assertEquals(1.0, camera.getClosestGamePiece().getFieldPosition().getX(), CLOSE);
  }

  /** The simulated camera, with the test's clock */
  static Camera simulated() {
    return new Camera.SimulatedCamera(
        () -> List.of(new Translation2d(4, 3)), () -> List.of(new Translation2d(2, 5))) {
      @Override
      protected double now() {
        return 100.0;
      }
    };
  }

  @Test
  public void theSimulatedCameraSeesWhatIsOnTheField() {
    Camera camera = simulated();
    assertFalse(camera.seesSomething());
    assertEquals("Needs the robot's pose: call usePose(...)", camera.getStatus());

    Camera withPose = simulated().usePose(() -> new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    assertTrue(withPose.hasClassesSet());
    Sighting robot = withPose.getClosestRobot();
    assertEquals(2.0, robot.getDistanceMeters(), CLOSE);
    assertEquals(-2.0, robot.getRobotRelativePosition().getY(), CLOSE); // to the right
    assertEquals(4.0, robot.getFieldPosition().getX(), CLOSE);
    Sighting piece = withPose.getClosestGamePiece();
    assertEquals(2.0, piece.getRobotRelativePosition().getX(), CLOSE); // straight ahead
  }
}
