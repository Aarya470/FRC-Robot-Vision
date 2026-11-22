package com.mercury1089.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * An object detection camera that sees robots and game pieces. Works with a Limelight, a
 * PhotonVision camera, or any other camera.
 *
 * <p>Set one up once, for example in RobotContainer:
 *
 * <pre>{@code
 * Camera frontCamera = Camera.limelight("limelight-front")
 *     .mountedAt(0.30, 0.0, 0.75, 15, 0)   // forward m, left m, height m, tilt down, facing
 *     .usePose(drivetrain::getPose)         // optional: lets it give field positions
 *     .redRobotClasses(1)                   // the model's class numbers
 *     .blueRobotClasses(2)
 *     .gamePieceClasses(0);
 * }</pre>
 *
 * <p>Then use it anywhere:
 *
 * <pre>{@code
 * if (frontCamera.seesRobotWithin(1.5)) { ... }
 * Sighting piece = frontCamera.getClosestGamePiece();   // null if it sees none
 * frontCamera.robotNearby(1.5).onTrue(...);             // a Trigger
 * SmartDashboard.putData("Front camera", frontCamera);   // status on the dashboard
 * }</pre>
 */
public abstract class Camera implements Sendable {
  /** Default height of the middle of a robot, in meters */
  public static final double DEFAULT_ROBOT_HEIGHT = 0.4;

  /** Default height of the middle of a game piece, in meters */
  public static final double DEFAULT_GAME_PIECE_HEIGHT = 0.08;

  // ---------------------------------------------------------------- settings

  private CameraMount mount = null;
  private Supplier<Pose2d> robotPose = null;

  // The robot's recent poses, so a frame can be placed with the pose from when it was taken
  // (frames arrive a little late, and the robot may have moved since)
  private final TimeInterpolatableBuffer<Pose2d> poseHistory =
      TimeInterpolatableBuffer.createBuffer(1.5);

  private final List<Integer> robotClasses = new ArrayList<>();
  private final List<Integer> redRobotClasses = new ArrayList<>();
  private final List<Integer> blueRobotClasses = new ArrayList<>();
  private final List<Integer> gamePieceClasses = new ArrayList<>();

  private double robotHeightMeters = DEFAULT_ROBOT_HEIGHT;
  private double gamePieceHeightMeters = DEFAULT_GAME_PIECE_HEIGHT;
  private double maxDistanceMeters = 8.0;
  private double maxAgeSeconds = 0.25;
  private double minConfidence = 0.5;

  // ---------------------------------------------------------------- what it sees

  private List<Sighting> sightings = new ArrayList<>();
  private String status = "No frames yet";
  private double lastReadTime = -1;

  // ---------------------------------------------------------------- making a camera

  /**
   * A Limelight running a neural detector pipeline. No extra library is needed.
   *
   * @param name the Limelight's name, such as "limelight" or "limelight-front"
   */
  public static Camera limelight(String name) {
    return LimelightCamera.create(name);
  }

  /**
   * A PhotonVision object detection camera. Needs the PhotonLib vendor library, with the same
   * version as PhotonVision on your coprocessor.
   *
   * @param name the camera's name in PhotonVision, exactly as typed there
   */
  public static Camera photonVision(String name) {
    try {
      return new PhotonVisionCamera(name);
    } catch (NoClassDefFoundError e) {
      // PhotonLib isn't installed, so Java can't find PhotonVision's classes
      throw new IllegalStateException(
          "Camera.photonVision(\"" + name + "\") needs the PhotonLib vendor library. Install it"
              + " with WPILib: Manage Vendor Libraries, or use Camera.limelight(...) instead.");
    }
  }

  /**
   * Any other camera that can tell you the angles to what it sees. Give a function that returns
   * everything it sees right now, or an empty list.
   *
   * <pre>{@code
   * Camera myCamera = Camera.fromAngles(() -> myCode.getTargets())
   *     .mountedAt(0.3, 0, 0.6, 20, 0);
   * }</pre>
   */
  public static Camera fromAngles(Supplier<List<CameraTarget>> targets) {
    return new AnglesCamera(targets);
  }

  /**
   * Any system that already knows where things are on the field, such as your own vision code.
   * The robot's pose must be given with {@link #usePose}. No mounting info is needed.
   *
   * @param type what the positions are: ThingType.ROBOT or ThingType.GAME_PIECE
   * @param positions a function that returns their field positions right now
   */
  public static Camera fromFieldPositions(
      ThingType type, Supplier<List<Translation2d>> positions) {
    return new FieldPositionsCamera(type, positions);
  }

  // ---------------------------------------------------------------- settings you can chain

  /**
   * Where the camera sits on the robot, measured from the robot's center on the floor.
   *
   * @param forwardMeters how far in front of the center (negative = behind)
   * @param leftMeters how far left of the center (negative = right)
   * @param heightMeters how high the lens is above the floor
   * @param tiltDownDegrees how far the camera is tilted down (0 = level)
   * @param facingDegrees which way it points: 0 = forward, 90 = left, 180 = backward, -90 = right
   * @return this camera, so settings can be chained
   */
  public Camera mountedAt(
      double forwardMeters,
      double leftMeters,
      double heightMeters,
      double tiltDownDegrees,
      double facingDegrees) {
    mount = new CameraMount(forwardMeters, leftMeters, heightMeters, tiltDownDegrees, facingDegrees);
    whenMountChanges();
    return this;
  }

  /**
   * Give the camera the robot's pose, usually {@code drivetrain::getPose}. Then sightings also
   * have field positions. Without it, distances and directions from the robot still work.
   */
  public Camera usePose(Supplier<Pose2d> robotPose) {
    this.robotPose = robotPose;
    return this;
  }

  /** These model classes are robots, when the model doesn't tell red from blue. */
  public Camera robotClasses(int... classIds) {
    addAll(robotClasses, classIds);
    return this;
  }

  /** These model classes are red alliance robots. */
  public Camera redRobotClasses(int... classIds) {
    addAll(redRobotClasses, classIds);
    return this;
  }

  /** These model classes are blue alliance robots. */
  public Camera blueRobotClasses(int... classIds) {
    addAll(blueRobotClasses, classIds);
    return this;
  }

  /** These model classes are game pieces. */
  public Camera gamePieceClasses(int... classIds) {
    addAll(gamePieceClasses, classIds);
    return this;
  }

  /** How high the middle of a robot is above the floor. Default 0.4 m. */
  public Camera robotHeight(double meters) {
    robotHeightMeters = meters;
    return this;
  }

  /** How high the middle of a game piece is above the floor. Default 0.08 m. */
  public Camera gamePieceHeight(double meters) {
    gamePieceHeightMeters = meters;
    return this;
  }

  /** Ignore anything farther than this from the robot. Default 8 m. */
  public Camera maxDistance(double meters) {
    maxDistanceMeters = meters;
    return this;
  }

  /** Forget what the camera saw after this many seconds. Default 0.25 s. */
  public Camera maxAge(double seconds) {
    maxAgeSeconds = seconds;
    return this;
  }

  /**
   * Ignore detections below this confidence, from 0 to 1. Default 0.5. Only PhotonVision reports
   * confidence. For a Limelight, set the threshold on its web page.
   */
  public Camera minConfidence(double confidence) {
    minConfidence = confidence;
    return this;
  }

  // ---------------------------------------------------------------- asking what it sees

  /** @return everything the camera sees right now */
  public List<Sighting> getSightings() {
    readIfNeeded();
    List<Sighting> fresh = new ArrayList<>();
    double now = now();
    for (Sighting sighting : sightings) {
      double age = now - sighting.getTimestampSeconds();
      if (age <= maxAgeSeconds) {
        fresh.add(sighting);
      }
    }
    return fresh;
  }

  /** @return every robot the camera sees right now */
  public List<Sighting> getRobots() {
    return onlyType(getSightings(), ThingType.ROBOT);
  }

  /** @return every game piece the camera sees right now */
  public List<Sighting> getGamePieces() {
    return onlyType(getSightings(), ThingType.GAME_PIECE);
  }

  /** @return true if the camera sees anything */
  public boolean seesSomething() {
    return !getSightings().isEmpty();
  }

  /** @return true if the camera sees a robot */
  public boolean seesRobot() {
    return !getRobots().isEmpty();
  }

  /** @return true if the camera sees a game piece */
  public boolean seesGamePiece() {
    return !getGamePieces().isEmpty();
  }

  /** @return true if anything the camera sees is within this many meters of the robot */
  public boolean seesSomethingWithin(double meters) {
    return isCloserThan(closest(getSightings()), meters);
  }

  /** @return true if a robot is within this many meters of the robot */
  public boolean seesRobotWithin(double meters) {
    return isCloserThan(getClosestRobot(), meters);
  }

  /** @return true if a game piece is within this many meters of the robot */
  public boolean seesGamePieceWithin(double meters) {
    return isCloserThan(getClosestGamePiece(), meters);
  }

  /** @return the closest thing the camera sees, or null if it sees nothing */
  public Sighting getClosest() {
    return closest(getSightings());
  }

  /** @return the closest robot, or null if the camera sees none */
  public Sighting getClosestRobot() {
    return closest(getRobots());
  }

  /** @return the closest game piece, or null if the camera sees none */
  public Sighting getClosestGamePiece() {
    return closest(getGamePieces());
  }

  /** A Trigger that is true while a robot is within this many meters. */
  public Trigger robotNearby(double meters) {
    return new Trigger(() -> seesRobotWithin(meters));
  }

  /** A Trigger that is true while the camera sees a game piece. */
  public Trigger gamePieceSeen() {
    return new Trigger(() -> seesGamePiece());
  }

  /**
   * @return "OK", "OK (no classes set: everything is OTHER)", or why nothing is coming through,
   *     such as "Not mounted: call mountedAt(...)" or "Ignored class 3". It also shows on the
   *     dashboard.
   */
  public String getStatus() {
    readIfNeeded();
    return status;
  }

  /** @return where the camera sits, or null if mountedAt(...) was not called */
  public CameraMount getMount() {
    return mount;
  }

  /**
   * @return true if any class numbers were given with robotClasses(...), redRobotClasses(...),
   *     blueRobotClasses(...) or gamePieceClasses(...). Without them, everything the camera sees
   *     is OTHER: never a robot or a game piece. Cameras made with fromFieldPositions(...) already
   *     know what they see, so they always return true.
   */
  public boolean hasClassesSet() {
    return !robotClasses.isEmpty()
        || !redRobotClasses.isEmpty()
        || !blueRobotClasses.isEmpty()
        || !gamePieceClasses.isEmpty();
  }

  /** @return the most this camera keeps a sighting, in seconds */
  public double getMaxAgeSeconds() {
    return maxAgeSeconds;
  }

  /** @return the lowest confidence a detection needs */
  public double getMinConfidence() {
    return minConfidence;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("MercuryVisionCamera");
    builder.addStringProperty("Status", () -> getStatus(), null);
    builder.addDoubleProperty("Robots seen", () -> getRobots().size(), null);
    builder.addDoubleProperty("Game pieces seen", () -> getGamePieces().size(), null);
    builder.addDoubleProperty("Closest robot m", () -> distanceOrMinusOne(getClosestRobot()), null);
    builder.addDoubleProperty(
        "Closest game piece m", () -> distanceOrMinusOne(getClosestGamePiece()), null);
  }

  // ---------------------------------------------------------------- for camera types

  /**
   * Read the camera's newest frame.
   *
   * @return the frame, or null if there is no new frame since last time
   */
  protected abstract CameraFrame readFrame();

  /** Called after mountedAt(...) changes the mount. */
  protected void whenMountChanges() {}

  /** @return the current time in seconds. Tests replace this. */
  protected double now() {
    return Timer.getFPGATimestamp();
  }

  /** @return the robot's pose, or null if usePose(...) was not called */
  protected Pose2d getRobotPoseOrNull() {
    if (robotPose == null) {
      return null;
    }
    return robotPose.get();
  }

  // ---------------------------------------------------------------- inside the camera

  /** Read a new frame at most once every 5 ms, however often the camera is asked. */
  private void readIfNeeded() {
    double now = now();
    if (lastReadTime >= 0 && now - lastReadTime < 0.005) {
      return;
    }
    lastReadTime = now;

    // Remember where the robot is now, for frames that arrive later
    Pose2d poseNow = getRobotPoseOrNull();
    if (poseNow != null) {
      poseHistory.addSample(now, poseNow);
    }

    CameraFrame frame = readFrame();
    if (frame == null) {
      // Same frame as before: keep the old sightings until they get too old.
      // Once they are all too old, the status shouldn't still say "OK".
      if (status.startsWith("OK") && !hasFreshSighting(now)) {
        status = "Sees nothing";
      }
      return;
    }
    sightings = makeSightings(frame, poseWhenTaken(frame.getTimestampSeconds()));
  }

  /**
   * @return the robot's pose when a frame was taken, or the pose now if there is no history for
   *     that time. Null without usePose(...).
   */
  private Pose2d poseWhenTaken(double frameTime) {
    Pose2d poseNow = getRobotPoseOrNull();
    if (poseNow == null) {
      return null;
    }
    Optional<Pose2d> then = poseHistory.getSample(frameTime);
    if (then.isPresent()) {
      return then.get();
    }
    return poseNow;
  }

  private boolean hasFreshSighting(double now) {
    for (Sighting sighting : sightings) {
      if (now - sighting.getTimestampSeconds() <= maxAgeSeconds) {
        return true;
      }
    }
    return false;
  }

  /** Turn one frame into sightings, and set the status. */
  List<Sighting> makeSightings(CameraFrame frame, Pose2d pose) {
    List<Sighting> result = new ArrayList<>();
    if (frame.getProblem() != null) {
      status = frame.getProblem();
      return result;
    }
    if (!frame.getAngleTargets().isEmpty() && mount == null) {
      status = "Not mounted: call mountedAt(...)";
      return result;
    }

    // Explains why nothing came through, if nothing does
    String reason = "Sees nothing";
    if (frame.getNote() != null) {
      reason = frame.getNote();
    }

    // Things seen as angles: work out where they are
    List<FoundTarget> found = new ArrayList<>(frame.getFoundTargets());
    for (CameraTarget target : frame.getAngleTargets()) {
      ThingType type = typeOf(target.getClassId());
      if (type == null) {
        reason = "Ignored class " + target.getClassId();
        continue;
      }
      double targetHeight = heightOf(type);
      Translation2d position =
          CameraMath.anglesToRobotRelative(
              mount, targetHeight, target.getTxDegrees(), target.getTyDegrees());
      if (position == null) {
        reason = whyMissed(targetHeight);
        continue;
      }
      found.add(new FoundTarget(target.getClassId(), type, position));
    }

    // Everything found: check it, then make a sighting
    for (FoundTarget target : found) {
      ThingType type = target.getType();
      if (type == null) {
        type = typeOf(target.getClassId());
      }
      if (type == null) {
        reason = "Ignored class " + target.getClassId();
        continue;
      }
      Translation2d robotRelative = target.getRobotRelativePosition();
      if (robotRelative.getNorm() > maxDistanceMeters) {
        reason = "Farther than maxDistance";
        continue;
      }
      Translation2d fieldPosition = null;
      if (pose != null) {
        fieldPosition = CameraMath.robotRelativeToField(pose, robotRelative);
      }
      TeamColor color = colorOf(target.getClassId());
      result.add(
          new Sighting(
              type,
              color,
              target.getClassId(),
              robotRelative,
              fieldPosition,
              frame.getTimestampSeconds()));
    }

    if (result.isEmpty()) {
      status = reason;
    } else if (!hasClassesSet()) {
      status = "OK (no classes set: everything is OTHER)";
    } else {
      status = "OK";
    }
    return result;
  }

  /** @return what kind of thing a class is, or null if it should be ignored */
  ThingType typeOf(int classId) {
    if (!hasClassesSet()) {
      return ThingType.OTHER;
    }
    if (robotClasses.contains(classId)
        || redRobotClasses.contains(classId)
        || blueRobotClasses.contains(classId)) {
      return ThingType.ROBOT;
    }
    if (gamePieceClasses.contains(classId)) {
      return ThingType.GAME_PIECE;
    }
    return null;
  }

  private TeamColor colorOf(int classId) {
    if (redRobotClasses.contains(classId)) {
      return TeamColor.RED;
    }
    if (blueRobotClasses.contains(classId)) {
      return TeamColor.BLUE;
    }
    return TeamColor.UNKNOWN;
  }

  private double heightOf(ThingType type) {
    if (type == ThingType.ROBOT) {
      return robotHeightMeters;
    }
    return gamePieceHeightMeters;
  }

  /**
   * Why a target couldn't be placed. A camera close to the target's height looks almost straight
   * across at it, so a tiny angle error makes a huge distance error. Mount it well above or below.
   */
  private String whyMissed(double targetHeight) {
    if (Math.abs(mount.getHeightMeters() - targetHeight) < 0.15) {
      return "Camera is level with the target's height: mount it higher or lower";
    }
    return "Can't place a target (check the camera's height and tilt)";
  }

  private static void addAll(List<Integer> list, int[] classIds) {
    for (int id : classIds) {
      list.add(id);
    }
  }

  private static List<Sighting> onlyType(List<Sighting> all, ThingType type) {
    List<Sighting> result = new ArrayList<>();
    for (Sighting sighting : all) {
      if (sighting.getType() == type) {
        result.add(sighting);
      }
    }
    return result;
  }

  private static Sighting closest(List<Sighting> list) {
    Sighting best = null;
    for (Sighting sighting : list) {
      if (best == null || sighting.getDistanceMeters() < best.getDistanceMeters()) {
        best = sighting;
      }
    }
    return best;
  }

  private static boolean isCloserThan(Sighting sighting, double meters) {
    return sighting != null && sighting.getDistanceMeters() <= meters;
  }

  private static double distanceOrMinusOne(Sighting sighting) {
    if (sighting == null) {
      return -1;
    }
    return sighting.getDistanceMeters();
  }
}
