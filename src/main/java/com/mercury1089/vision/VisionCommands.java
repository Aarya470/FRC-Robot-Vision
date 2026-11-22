package com.mercury1089.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;

/** Ready-made commands that use a camera. */
public class VisionCommands {
  private VisionCommands() {}

  /** Waits until the camera sees a game piece. */
  public static Command waitUntilSeesGamePiece(Camera camera) {
    return Commands.waitUntil(() -> camera.seesGamePiece());
  }

  /** Waits until the camera sees a robot. */
  public static Command waitUntilSeesRobot(Camera camera) {
    return Commands.waitUntil(() -> camera.seesRobot());
  }

  /** Waits until no robot is within this many meters, for example before driving through. */
  public static Command waitUntilNoRobotWithin(Camera camera, double meters) {
    return Commands.waitUntil(() -> !camera.seesRobotWithin(meters));
  }

  /**
   * Drives to the closest game piece the camera sees, facing it. Run your intake command at the
   * same time, for example: {@code
   * VisionCommands.driveToClosestGamePiece(...).deadlineFor(yourIntakeCommand)}.
   *
   * @param camera the camera that sees game pieces
   * @param driveRobotRelative your drivetrain's method that takes robot-relative ChassisSpeeds, for
   *     example {@code drivetrain::driveRobotRelative}
   * @param maxSpeed the fastest to drive, in meters per second
   * @param intakeOnBack true if the intake is on the back, so the robot backs onto the piece
   * @param arriveDistance stop when the piece is this close to the robot's center, in meters.
   *     About half your robot's length with bumpers, such as 0.5.
   * @param drivetrain your drivetrain subsystem
   */
  public static Command driveToClosestGamePiece(
      Camera camera,
      Consumer<ChassisSpeeds> driveRobotRelative,
      double maxSpeed,
      boolean intakeOnBack,
      double arriveDistance,
      Subsystem drivetrain) {
    return new DriveToGamePieceCommand(
        camera, driveRobotRelative, maxSpeed, intakeOnBack, arriveDistance, drivetrain);
  }
}
