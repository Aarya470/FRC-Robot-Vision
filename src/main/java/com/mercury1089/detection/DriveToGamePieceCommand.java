package com.mercury1089.detection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;

/**
 * Drives toward a game piece the camera sees, turning to face it. Works with any drivetrain that
 * can drive forward and turn.
 *
 * <p>How it works:
 *
 * <ol>
 *   <li>It picks the closest game piece, then sticks with that one piece (it doesn't hop to
 *       another piece that is a bit closer). With usePose(...) on the camera, it follows the piece
 *       by its field position, which works best.
 *   <li>It drives toward the piece, slowing down as it gets close.
 *   <li>Near the robot, the camera often can't see the piece any more (it is under the bumper). If
 *       the piece was close and nearly straight ahead of the intake when the camera lost it, the
 *       robot keeps driving slowly straight at it for as long as that should take, then stops.
 *   <li>It ends when the piece is within the arrive distance, after that last blind push, or if
 *       the camera loses a far away piece for half a second.
 * </ol>
 *
 * <p>While it can see the piece, it keeps going. If something can stop the robot reaching it (a
 * wall, another robot), add a time limit: {@code .withTimeout(3)}.
 *
 * <pre>{@code
 * new DriveToGamePieceCommand(
 *         intakeCamera,
 *         drivetrain::driveRobotRelative, // your method that takes robot-relative ChassisSpeeds
 *         2.0,                            // top speed, meters per second
 *         false,                          // true if the intake is on the back
 *         0.5,                            // stop when the piece is this close to the robot's center
 *         drivetrain)
 *     .deadlineFor(yourIntakeCommand);
 * }</pre>
 */
public class DriveToGamePieceCommand extends Command {
  /** How fast to turn for each radian the piece is off to the side, in radians per second */
  static final double TURN_GAIN = 3.0;

  /** How fast to drive for each meter away, in meters per second */
  static final double DRIVE_GAIN = 1.5;

  /** The fastest the robot turns, in radians per second */
  static final double MAX_TURN_SPEED = 3.0;

  /** Give up after the camera has lost a far away piece for this long, in seconds */
  static final double LOST_TIMEOUT = 0.5;

  /** A piece lost closer than this is driven onto blind, in meters */
  static final double CLOSE_DISTANCE = 1.2;

  /** How fast to drive onto a piece the camera can't see any more, in meters per second */
  static final double BLIND_SPEED = 0.75;

  /** A piece this close to where the locked piece should be counts as the same piece, in meters */
  static final double SAME_PIECE_DISTANCE = 0.5;

  /** A lost piece is only driven onto blind if it was within this angle of straight ahead */
  static final double BLIND_MAX_ANGLE_DEGREES = 15.0;

  private final Camera camera;
  private final Consumer<ChassisSpeeds> driveRobotRelative;
  private final double maxSpeed;
  private final boolean intakeOnBack;
  private final double arriveDistance;

  private boolean arrived = false;

  // When the camera last saw our piece (or when the blind push started), in seconds
  private double lastSeenTime = 0.0;

  // The piece we are driving to, when the camera last saw it: from the robot's center, and on the
  // field (null if the camera has no pose)
  private Translation2d lockedPiece = null;
  private Translation2d lockedPieceOnField = null;

  // Driving blind onto a close piece the camera lost
  private boolean drivingBlind = false;
  private double blindSeconds = 0.0;

  /**
   * @param camera the camera that sees game pieces
   * @param driveRobotRelative your drivetrain's method that takes robot-relative ChassisSpeeds
   * @param maxSpeed the fastest to drive, in meters per second
   * @param intakeOnBack true if the intake is on the back, so the robot backs onto the piece
   * @param arriveDistance stop when the piece is this close to the robot's center, in meters. Use
   *     about half your robot's length with bumpers, where the intake grabs it.
   * @param drivetrain your drivetrain subsystem
   */
  public DriveToGamePieceCommand(
      Camera camera,
      Consumer<ChassisSpeeds> driveRobotRelative,
      double maxSpeed,
      boolean intakeOnBack,
      double arriveDistance,
      Subsystem drivetrain) {
    this.camera = camera;
    this.driveRobotRelative = driveRobotRelative;
    this.maxSpeed = maxSpeed;
    this.intakeOnBack = intakeOnBack;
    this.arriveDistance = arriveDistance;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    arrived = false;
    lockedPiece = null;
    lockedPieceOnField = null;
    drivingBlind = false;
    lastSeenTime = now();
  }

  @Override
  public void execute() {
    Sighting seen = findLockedPiece();

    if (seen != null) {
      // The camera sees our piece
      Translation2d piece = seen.getRobotRelativePosition();
      lockedPiece = piece;
      lockedPieceOnField = seen.getFieldPosition();
      drivingBlind = false;
      lastSeenTime = now();
      if (piece.getNorm() <= arriveDistance) {
        arrived = true;
        driveRobotRelative.accept(new ChassisSpeeds());
        return;
      }
      driveRobotRelative.accept(speedsToward(piece, maxSpeed, intakeOnBack));
      return;
    }

    // The camera can't see our piece right now. Drive on blind only if it was close and nearly
    // straight ahead of the intake.
    boolean wasClose = lockedPiece != null && lockedPiece.getNorm() < CLOSE_DISTANCE;
    boolean wasAhead =
        lockedPiece != null && Math.abs(angleFromIntake(lockedPiece)) <= BLIND_MAX_ANGLE_DEGREES;
    if (!wasClose || !wasAhead) {
      driveRobotRelative.accept(new ChassisSpeeds()); // stop and wait for it to come back
      return;
    }

    if (!drivingBlind) {
      // Start a blind push: long enough to cover the distance left to the arrive distance
      drivingBlind = true;
      double distanceLeft = Math.max(0.0, lockedPiece.getNorm() - arriveDistance);
      blindSeconds = distanceLeft / blindSpeed();
      lastSeenTime = now();
    }
    if (now() - lastSeenTime >= blindSeconds) {
      arrived = true;
      driveRobotRelative.accept(new ChassisSpeeds());
      return;
    }
    driveRobotRelative.accept(blindSpeeds());
  }

  @Override
  public boolean isFinished() {
    if (arrived) {
      return true;
    }
    return !drivingBlind && now() - lastSeenTime >= LOST_TIMEOUT;
  }

  @Override
  public void end(boolean interrupted) {
    driveRobotRelative.accept(new ChassisSpeeds());
  }

  /**
   * @return the piece we are driving to, or null if the camera doesn't see it right now. The first
   *     time, this is the closest piece.
   */
  private Sighting findLockedPiece() {
    if (lockedPiece == null) {
      return camera.getClosestGamePiece();
    }

    // The piece the camera sees nearest to where our piece was last
    Sighting best = null;
    double bestDistance = SAME_PIECE_DISTANCE;
    for (Sighting sighting : camera.getGamePieces()) {
      double distance;
      if (lockedPieceOnField != null && sighting.getFieldPosition() != null) {
        // Field positions don't change as the robot moves, so this is the best match
        distance = sighting.getFieldPosition().getDistance(lockedPieceOnField);
      } else {
        distance = sighting.getRobotRelativePosition().getDistance(lockedPiece);
      }
      if (distance <= bestDistance) {
        best = sighting;
        bestDistance = distance;
      }
    }

    // Without field positions, the robot may have moved too much to match. Then take the closest.
    if (best == null && lockedPieceOnField == null) {
      return camera.getClosestGamePiece();
    }
    return best;
  }

  /** @return how far a piece is from straight ahead of the intake, in degrees */
  private double angleFromIntake(Translation2d piece) {
    double angle = Math.toDegrees(Math.atan2(piece.getY(), piece.getX()));
    if (intakeOnBack) {
      angle = MathUtil.inputModulus(angle - 180.0, -180.0, 180.0);
    }
    return angle;
  }

  /** @return the current time in seconds. Tests replace this. */
  double now() {
    return Timer.getFPGATimestamp();
  }

  private double blindSpeed() {
    return Math.min(BLIND_SPEED, maxSpeed);
  }

  /**
   * Drive slowly straight ahead (or straight back, if the intake is on the back), without turning.
   * The robot was already facing the piece when the camera lost it.
   */
  private ChassisSpeeds blindSpeeds() {
    double speed = blindSpeed();
    if (intakeOnBack) {
      speed = -speed;
    }
    return new ChassisSpeeds(speed, 0.0, 0.0);
  }

  /**
   * The speeds that drive toward a target and turn to face it.
   *
   * @param target the target from the robot's center (x forward, y left), in meters
   * @param maxSpeed the fastest to drive, in meters per second
   * @param intakeOnBack true to face the target with the back of the robot
   * @return robot-relative speeds
   */
  static ChassisSpeeds speedsToward(Translation2d target, double maxSpeed, boolean intakeOnBack) {
    // Which way the target is, from the side of the robot that should face it
    double angleToTarget = Math.atan2(target.getY(), target.getX());
    if (intakeOnBack) {
      angleToTarget = MathUtil.angleModulus(angleToTarget - Math.PI);
    }

    // Turn toward it
    double turnSpeed = TURN_GAIN * angleToTarget;
    turnSpeed = MathUtil.clamp(turnSpeed, -MAX_TURN_SPEED, MAX_TURN_SPEED);

    // Drive toward it, slower while it is still off to the side
    double driveSpeed = Math.min(maxSpeed, DRIVE_GAIN * target.getNorm());
    driveSpeed = driveSpeed * Math.max(0.0, Math.cos(angleToTarget));
    if (intakeOnBack) {
      driveSpeed = -driveSpeed;
    }
    return new ChassisSpeeds(driveSpeed, 0.0, turnSpeed);
  }
}
