package com.mercury1089.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class CameraMathTest {
  private static final double CLOSE = 1e-6;

  @Test
  public void straightAheadOnTheFloor() {
    // 1 m up, tilted 45 degrees down, target in the middle of the image
    CameraMount mount = new CameraMount(0, 0, 1.0, 45, 0);
    Translation2d p = CameraMath.anglesToRobotRelative(mount, 0, 0, 0);
    assertEquals(1.0, p.getX(), CLOSE);
    assertEquals(0.0, p.getY(), CLOSE);
  }

  @Test
  public void aboveTheMiddleIsFarther() {
    // 30 down plus 10 up in the image = 20 degrees below level
    CameraMount mount = new CameraMount(0, 0, 1.0, 30, 0);
    Translation2d p = CameraMath.anglesToRobotRelative(mount, 0, 0, 10);
    assertEquals(1.0 / Math.tan(Math.toRadians(20)), p.getX(), CLOSE);
  }

  @Test
  public void positiveTxIsToTheRight() {
    CameraMount mount = new CameraMount(0, 0, 1.0, 45, 0);
    Translation2d p = CameraMath.anglesToRobotRelative(mount, 0, 20, 0);
    assertTrue(p.getY() < 0);
    assertTrue(p.getX() > 0);
  }

  @Test
  public void backwardCameraBehindTheCenter() {
    CameraMount mount = new CameraMount(-0.3, 0, 1.0, 45, 180);
    Translation2d p = CameraMath.anglesToRobotRelative(mount, 0, 0, 0);
    assertEquals(-1.3, p.getX(), CLOSE);
    assertEquals(0.0, p.getY(), CLOSE);
  }

  @Test
  public void targetHeightShortensTheDistance() {
    CameraMount mount = new CameraMount(0, 0, 1.0, 45, 0);
    Translation2d p = CameraMath.anglesToRobotRelative(mount, 0.5, 0, 0);
    assertEquals(0.5, p.getX(), CLOSE);
  }

  @Test
  public void lowCameraLooksUpAtARobot() {
    CameraMount mount = new CameraMount(0, 0, 0.13, 0, 0);
    double ty = Math.toDegrees(Math.atan(0.27 / 2.0));
    Translation2d p = CameraMath.anglesToRobotRelative(mount, 0.4, 0, ty);
    assertEquals(2.0, p.getX(), CLOSE);
  }

  @Test
  public void aboveTheHorizonGivesNull() {
    assertNull(CameraMath.anglesToRobotRelative(new CameraMount(0, 0, 1.0, 0, 0), 0, 0, 5));
    assertNull(CameraMath.anglesToRobotRelative(new CameraMount(0, 0, 1.0, 10, 0), 0, 0, 10));
  }

  /** Where a floor point shows up in a pinhole camera, worked out the other way round */
  private static double[] pixelOf(Translation2d floorPoint, CameraMount m, double f, double cx, double cy) {
    double dx = floorPoint.getX() - m.getForwardMeters();
    double dy = floorPoint.getY() - m.getLeftMeters();
    double dz = -m.getHeightMeters();
    double yaw = Math.toRadians(m.getFacingDegrees());
    double ax = dx * Math.cos(yaw) + dy * Math.sin(yaw);
    double ay = -dx * Math.sin(yaw) + dy * Math.cos(yaw);
    double tilt = Math.toRadians(m.getTiltDownDegrees());
    double camX = ax * Math.cos(tilt) - dz * Math.sin(tilt);
    double camZ = ax * Math.sin(tilt) + dz * Math.cos(tilt);
    return new double[] {cx - f * ay / camX, cy - f * camZ / camX};
  }

  @Test
  public void calibratedPixelsAreExactEvenOffCenter() {
    CameraMount mount = new CameraMount(0.3, 0.1, 0.75, 15, 30);
    Translation2d[] points = {
      new Translation2d(3.0, 0.5), new Translation2d(4.0, -1.0), new Translation2d(2.0, 2.0)
    };
    for (Translation2d point : points) {
      double[] pixel = pixelOf(point, mount, 500, 320, 240);
      Translation2d got =
          CameraMath.floorPixelToRobotRelative(mount, 500, 500, 320, 240, pixel[0], pixel[1]);
      assertNotNull(got);
      assertEquals(point.getX(), got.getX(), CLOSE);
      assertEquals(point.getY(), got.getY(), CLOSE);
    }
  }

  @Test
  public void photonPitchTurnsIntoLimelightTy() {
    // A target 3 m ahead and 1.2 m right, 0.35 m below a level camera
    double nx = -1.2 / 3.0;
    double ny = -0.35 / 3.0;
    double yaw = Math.toDegrees(Math.atan(-nx));
    double photonPitch = Math.toDegrees(Math.atan(ny * Math.cos(Math.toRadians(yaw))));
    double ty = CameraMath.photonPitchToTy(yaw, photonPitch);
    assertEquals(Math.toDegrees(Math.atan(ny)), ty, 1e-9);
  }

  @Test
  public void robotRelativeToField() {
    Pose2d robot = new Pose2d(5, 3, Rotation2d.fromDegrees(90));
    Translation2d field = CameraMath.robotRelativeToField(robot, new Translation2d(1, 0));
    assertEquals(5.0, field.getX(), CLOSE);
    assertEquals(4.0, field.getY(), CLOSE);
  }
}
