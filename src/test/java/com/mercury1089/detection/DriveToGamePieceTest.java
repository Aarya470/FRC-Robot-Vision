package com.mercury1089.detection;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class DriveToGamePieceTest {
  /** A clock the tests move by hand */
  static double time = 100.0;

  /** A camera that sees game pieces wherever the test puts them (from the robot's center) */
  static class TestCamera extends Camera {
    List<Translation2d> pieces = new ArrayList<>();

    TestCamera() {
      mountedAt(0, 0, 0, 0, 0);
      gamePieceClasses(0);
    }

    @Override
    Frame readFrame() {
      Frame frame = new Frame(time);
      for (Translation2d piece : pieces) {
        frame.detections.add(new Detection(0, piece));
      }
      return frame;
    }

    @Override
    protected double now() {
      return time;
    }
  }

  TestCamera camera = new TestCamera();
  ChassisSpeeds lastSpeeds = new ChassisSpeeds();
  DriveToGamePieceCommand command =
      new DriveToGamePieceCommand(camera, (speeds) -> lastSpeeds = speeds, 2.0, false, 0.5, new Subsystem() {}) {
        @Override
        double now() {
          return time;
        }
      };

  /** Run the command for one 20 ms loop */
  void loop() {
    time += 0.02;
    command.execute();
  }

  @Test
  public void stopsWhenThePieceIsWithinTheArriveDistance() {
    camera.pieces.add(new Translation2d(0.45, 0));
    command.initialize();
    loop();
    assertTrue(command.isFinished());
  }

  @Test
  public void keepsDrivingOntoAClosePieceTheCameraLoses() {
    camera.pieces.add(new Translation2d(0.95, 0));
    command.initialize();
    loop();
    assertTrue(lastSpeeds.vxMetersPerSecond > 0);

    camera.pieces.clear(); // the piece went under the bumper
    double blindTime = 0;
    while (!command.isFinished() && blindTime < 2) {
      loop();
      blindTime += 0.02;
      if (!command.isFinished()) {
        assertEquals(0.75, lastSpeeds.vxMetersPerSecond, 1e-9, "slow and straight");
        assertEquals(0.0, lastSpeeds.omegaRadiansPerSecond, 1e-9);
      }
    }
    assertTrue(command.isFinished());
    // 0.45 m left to go at 0.75 m/s is 0.6 s
    assertEquals(0.6, blindTime, 0.05);
  }

  @Test
  public void stopsAndGivesUpOnAFarPieceTheCameraLoses() {
    camera.pieces.add(new Translation2d(3, 0));
    command.initialize();
    loop();
    camera.pieces.clear();
    loop();
    assertEquals(0.0, lastSpeeds.vxMetersPerSecond, 1e-9);
    assertFalse(command.isFinished());
    for (int i = 0; i < 30; i++) {
      loop();
    }
    assertTrue(command.isFinished());
  }

  @Test
  public void sticksWithTheFirstPieceInsteadOfHopping() {
    camera.pieces.add(new Translation2d(2, 0));
    command.initialize();
    loop();

    // Our piece is now a bit closer, and another piece shows up even closer, off to the left
    camera.pieces.clear();
    camera.pieces.add(new Translation2d(1.9, 0));
    camera.pieces.add(new Translation2d(1.0, 1.0));
    loop();
    assertEquals(0.0, lastSpeeds.omegaRadiansPerSecond, 1e-9, "still heading straight at the first piece");
  }

  @Test
  public void doesNotPushBlindAtAPieceOffToTheSide() {
    camera.pieces.add(new Translation2d(0.8, 0.7)); // close, but about 40 degrees to the left
    command.initialize();
    loop();
    camera.pieces.clear();
    loop();
    assertEquals(0.0, lastSpeeds.vxMetersPerSecond, 1e-9, "stops instead of pushing the wrong way");
    for (int i = 0; i < 30; i++) {
      loop();
    }
    assertTrue(command.isFinished());
  }

  @Test
  public void followsThePieceByFieldPositionWhenItHasAPose() {
    // The robot is at (0, 0) facing forward, then drives 1 m forward between frames
    Pose2d[] robot = {new Pose2d()};
    camera.usePose(() -> robot[0]);
    camera.pieces.add(new Translation2d(3, 0));
    command.initialize();
    loop();

    // Our piece is now 2 m ahead (it didn't move). Another piece is 2.6 m ahead, which is
    // close to where ours was relative to the robot before.
    robot[0] = new Pose2d(1, 0, Rotation2d.kZero);
    camera.pieces.clear();
    camera.pieces.add(new Translation2d(2, 0));
    camera.pieces.add(new Translation2d(2.6, 0.3));
    loop();
    assertEquals(0.0, lastSpeeds.omegaRadiansPerSecond, 1e-9, "still heading at our piece");
  }

  @Test
  public void drivesStraightAtAPieceAhead() {
    ChassisSpeeds s = DriveToGamePieceCommand.speedsToward(new Translation2d(2, 0), 1.0, false);
    assertEquals(1.0, s.vxMetersPerSecond, 1e-9);
    assertEquals(0.0, s.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  public void turnsLeftTowardAPieceOnTheLeft() {
    ChassisSpeeds s = DriveToGamePieceCommand.speedsToward(new Translation2d(1, 1), 1.0, false);
    assertTrue(s.omegaRadiansPerSecond > 0);
    assertTrue(s.vxMetersPerSecond > 0);
  }

  @Test
  public void doesNotDriveAwayFromAPieceBehind() {
    ChassisSpeeds s = DriveToGamePieceCommand.speedsToward(new Translation2d(-2, 0.1), 1.0, false);
    assertEquals(0.0, s.vxMetersPerSecond, 1e-9);
  }

  @Test
  public void backsOntoAPieceBehindWhenTheIntakeIsOnTheBack() {
    ChassisSpeeds s = DriveToGamePieceCommand.speedsToward(new Translation2d(-2, 0), 1.0, true);
    assertEquals(-1.0, s.vxMetersPerSecond, 1e-9);
    assertEquals(0.0, s.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  public void slowsDownWhenClose() {
    ChassisSpeeds s = DriveToGamePieceCommand.speedsToward(new Translation2d(0.4, 0), 3.0, false);
    assertEquals(0.6, s.vxMetersPerSecond, 1e-9);
  }
}
