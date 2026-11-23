package com.mercury1089.detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/**
 * A camera that sees robots and game pieces.
 *
 * <pre>{@code
 * Camera camera = Camera.limelight("limelight-front")
 *     .mountedAt(0.3, 0.0, 0.5, 15, 0)   // forward, left, height (m), tilted down, facing (deg)
 *     .redRobotClasses(1)
 *     .blueRobotClasses(2)
 *     .gamePieceClasses(0)
 *     .usePose(drivetrain::getPose);     // optional: also gives field positions
 *
 * if (camera.seesGamePiece()) { ... }
 * }</pre>
 *
 * <p>How it works: the camera says which way each thing is (an angle left-right and up-down). We
 * know how high the camera is and how high the thing is, so we follow that line from the camera
 * until it reaches the thing's height. That is where the thing is.
 */
public abstract class Camera implements Sendable {
  /** Sightings older than this are forgotten, in seconds */
  static final double MAX_AGE_SECONDS = 0.25;

  // Where the camera is on the robot
  private boolean mounted = false;
  private double mountForward;
  private double mountLeft;
  private double mountHeight;
  private double mountTiltDown;
  private double mountFacing;

  // Which class numbers are robots and game pieces
  private final Set<Integer> robotClasses = new HashSet<>();
  private final Set<Integer> redRobotClasses = new HashSet<>();
  private final Set<Integer> blueRobotClasses = new HashSet<>();
  private final Set<Integer> gamePieceClasses = new HashSet<>();

  private double robotHeight = 0.4;
  private double gamePieceHeight = 0.08;
  private double maxDistance = 8.0;
  private Supplier<Pose2d> robotPose = null;

  // What the camera sees right now
  private List<Sighting> sightings = new ArrayList<>();
  private String status = "No pictures yet";
  private double lastUpdateTime = -1;

  // The robot's recent poses, so a picture can be placed using where the robot was when the
  // picture was taken (pictures arrive a little late)
  private final TimeInterpolatableBuffer<Pose2d> poseHistory =
      TimeInterpolatableBuffer.createBuffer(1.5);

  // Only the cameras in this library: make one with Camera.limelight(...) and friends
  Camera() {}

  // ---------------------------------------------------------------- making a camera

  /**
   * A Limelight running a neural detector pipeline.
   *
   * @param name the Limelight's name, such as "limelight" or "limelight-front"
   */
  public static Camera limelight(String name) {
    return new LimelightCamera(name);
  }

  /**
   * A PhotonVision camera running an object detection pipeline. Needs the PhotonLib vendor library.
   *
   * @param name the camera's name in PhotonVision
   */
  public static Camera photonVision(String name) {
    try {
      return new PhotonVisionCamera(name);
    } catch (NoClassDefFoundError e) {
      throw new IllegalStateException(
          "Camera.photonVision(\"" + name + "\") needs the PhotonLib vendor library. Install it"
              + " with WPILib: Manage Vendor Libraries, or use Camera.limelight(...) instead.");
    }
  }

  /**
   * A pretend camera for simulation: it sees robots and game pieces where you say they are on the
   * field. It needs {@link #usePose}. Don't call mountedAt or the class settings on it.
   *
   * @param robots where robots are on the field
   * @param gamePieces where game pieces are on the field
   */
  public static Camera simulated(
      Supplier<List<Translation2d>> robots, Supplier<List<Translation2d>> gamePieces) {
    return new SimulatedCamera(robots, gamePieces);
  }

  // ---------------------------------------------------------------- settings

  /**
   * Where the camera is on the robot. Needed before it can see anything.
   *
   * @param forwardMeters how far in front of the robot's center (negative = behind)
   * @param leftMeters how far left of the robot's center (negative = right)
   * @param heightMeters how high the lens is above the floor
   * @param tiltDownDegrees how far it is tilted down (negative = up)
   * @param facingDegrees which way it faces: 0 = forward, 90 = left, 180 = backward, -90 = right
   */
  public Camera mountedAt(
      double forwardMeters,
      double leftMeters,
      double heightMeters,
      double tiltDownDegrees,
      double facingDegrees) {
    mounted = true;
    mountForward = forwardMeters;
    mountLeft = leftMeters;
    mountHeight = heightMeters;
    mountTiltDown = tiltDownDegrees;
    mountFacing = facingDegrees;
    return this;
  }

  /** Class numbers that are robots, when the model can't tell their color. */
  public Camera robotClasses(int... classIds) {
    addAll(robotClasses, classIds);
    return this;
  }

  /** Class numbers that are red robots. */
  public Camera redRobotClasses(int... classIds) {
    addAll(redRobotClasses, classIds);
    return this;
  }

  /** Class numbers that are blue robots. */
  public Camera blueRobotClasses(int... classIds) {
    addAll(blueRobotClasses, classIds);
    return this;
  }

  /** Class numbers that are game pieces. */
  public Camera gamePieceClasses(int... classIds) {
    addAll(gamePieceClasses, classIds);
    return this;
  }

  /** How high the middle of a robot is above the floor, in meters. Default 0.4. */
  public Camera robotHeight(double meters) {
    robotHeight = meters;
    return this;
  }

  /** How high the middle of a game piece is above the floor, in meters. Default 0.08. */
  public Camera gamePieceHeight(double meters) {
    gamePieceHeight = meters;
    return this;
  }

  /** Ignore things farther than this from the robot's center, in meters. Default 8. */
  public Camera maxDistance(double meters) {
    maxDistance = meters;
    return this;
  }

  /** Give the camera the robot's pose, so sightings also get field positions. */
  public Camera usePose(Supplier<Pose2d> poseSupplier) {
    robotPose = poseSupplier;
    return this;
  }

  /** @return true if any class numbers were given. Without them, the camera sees nothing. */
  public boolean hasClassesSet() {
    return !robotClasses.isEmpty()
        || !redRobotClasses.isEmpty()
        || !blueRobotClasses.isEmpty()
        || !gamePieceClasses.isEmpty();
  }

  // ---------------------------------------------------------------- what it sees

  /** @return every robot it sees right now */
  public List<Sighting> getRobots() {
    return getAll(Sighting.Type.ROBOT);
  }

  /** @return every game piece it sees right now */
  public List<Sighting> getGamePieces() {
    return getAll(Sighting.Type.GAME_PIECE);
  }

  /** @return true if it sees a robot or a game piece */
  public boolean seesSomething() {
    update();
    return !sightings.isEmpty();
  }

  /** @return true if it sees a robot */
  public boolean seesRobot() {
    return !getRobots().isEmpty();
  }

  /** @return true if it sees a game piece */
  public boolean seesGamePiece() {
    return !getGamePieces().isEmpty();
  }

  /** @return true if it sees a robot closer than this to the robot's center, in meters */
  public boolean seesRobotWithin(double meters) {
    Sighting closest = getClosestRobot();
    return closest != null && closest.getDistanceMeters() <= meters;
  }

  /** @return true if it sees a game piece closer than this to the robot's center, in meters */
  public boolean seesGamePieceWithin(double meters) {
    Sighting closest = getClosestGamePiece();
    return closest != null && closest.getDistanceMeters() <= meters;
  }

  /** @return the closest robot it sees, or null if none */
  public Sighting getClosestRobot() {
    return closest(getRobots());
  }

  /** @return the closest game piece it sees, or null if none */
  public Sighting getClosestGamePiece() {
    return closest(getGamePieces());
  }

  /**
   * @return "OK", or why it sees nothing, such as "Sees nothing" or "Not mounted: call
   *     mountedAt(...)". It also shows on the dashboard.
   */
  public String getStatus() {
    update();
    return status;
  }

  // ---------------------------------------------------------------- commands and triggers

  /** A trigger that is true while a robot is closer than this, in meters. */
  public Trigger robotNearby(double meters) {
    return new Trigger(() -> seesRobotWithin(meters));
  }

  /** A trigger that is true while it sees a game piece. */
  public Trigger gamePieceSeen() {
    return new Trigger(this::seesGamePiece);
  }

  /** A command that waits until it sees a game piece. */
  public Command waitUntilSeesGamePiece() {
    return Commands.waitUntil(this::seesGamePiece);
  }

  /** A command that waits until it sees a robot. */
  public Command waitUntilSeesRobot() {
    return Commands.waitUntil(this::seesRobot);
  }

  /** A command that waits until no robot is closer than this, in meters. */
  public Command waitUntilNoRobotWithin(double meters) {
    return Commands.waitUntil(() -> !seesRobotWithin(meters));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Status", this::getStatus, null);
    builder.addIntegerProperty("Robots seen", () -> getRobots().size(), null);
    builder.addIntegerProperty("Game pieces seen", () -> getGamePieces().size(), null);
  }

  // ---------------------------------------------------------------- for each kind of camera
  // (Limelight, PhotonVision and simulated each read pictures their own way)

  /** One thing in a camera picture */
  static class Detection {
    final int classId;
    final double txDegrees; // left-right angle from the picture's center, positive = right
    final double tyDegrees; // up-down angle from the picture's center, positive = up
    // Where it is from the robot's center, if already known (only the simulated camera knows
    // this). When this is null, the position is worked out from the angles.
    final Translation2d position;

    /** A thing seen at these angles. */
    Detection(int classId, double txDegrees, double tyDegrees) {
      this.classId = classId;
      this.txDegrees = txDegrees;
      this.tyDegrees = tyDegrees;
      this.position = null;
    }

    /** A thing already known to be here, from the robot's center. */
    Detection(int classId, Translation2d position) {
      this.classId = classId;
      this.txDegrees = 0;
      this.tyDegrees = 0;
      this.position = position;
    }
  }

  /** One camera picture: when it was taken, and what is in it (or what went wrong) */
  static class Frame {
    final double timestampSeconds;
    final List<Detection> detections = new ArrayList<>();
    String problem = null;

    Frame(double timestampSeconds) {
      this.timestampSeconds = timestampSeconds;
    }
  }

  /**
   * Get the camera's newest picture.
   *
   * @return the picture, or null if there is no new one since last time
   */
  abstract Frame readFrame();

  /** @return the time now, in seconds. Tests replace this. */
  protected double now() {
    return Timer.getFPGATimestamp();
  }

  // ---------------------------------------------------------------- inside

  /**
   * Read the newest picture. Asking many times in one robot loop only reads once: it skips if it
   * read less than 5 ms ago.
   */
  private void update() {
    double now = now();
    if (lastUpdateTime >= 0 && now - lastUpdateTime < 0.005) {
      return;
    }
    lastUpdateTime = now;

    if (robotPose != null) {
      poseHistory.addSample(now, robotPose.get());
    }

    Frame frame = readFrame();
    if (frame != null) {
      sightings = makeSightings(frame);
    }

    // Forget sightings that are too old
    sightings.removeIf(sighting -> now - sighting.getTimestampSeconds() > MAX_AGE_SECONDS);
    if (status.equals("OK") && sightings.isEmpty()) {
      status = "Sees nothing";
    }
  }

  /** Turn a picture into sightings, and set the status. */
  List<Sighting> makeSightings(Frame frame) {
    List<Sighting> found = new ArrayList<>();
    if (frame.problem != null) {
      status = frame.problem;
      return found;
    }
    if (!hasClassesSet()) {
      status = "No classes set: call robotClasses(...) or gamePieceClasses(...)";
      return found;
    }
    if (!mounted) {
      status = "Not mounted: call mountedAt(...)";
      return found;
    }

    Pose2d pose = poseWhenTaken(frame.timestampSeconds);
    String whyNothing = "Sees nothing";
    for (Detection detection : frame.detections) {
      Sighting.Type type = typeOf(detection.classId);
      if (type == null) {
        whyNothing = "Ignored class " + detection.classId;
        continue;
      }

      Translation2d position = detection.position;
      if (position == null) {
        double height = (type == Sighting.Type.ROBOT) ? robotHeight : gamePieceHeight;
        position = anglesToPosition(detection.txDegrees, detection.tyDegrees, height);
      }
      if (position == null) {
        whyNothing =
            "Can't place what it sees: check the height and tilt in mountedAt(...), and mount it"
                + " well above or below the things it looks for";
        continue;
      }
      if (position.getNorm() > maxDistance) {
        whyNothing = "Farther than maxDistance";
        continue;
      }

      Translation2d fieldPosition = null;
      if (pose != null) {
        fieldPosition = pose.getTranslation().plus(position.rotateBy(pose.getRotation()));
      }
      found.add(
          new Sighting(
              type,
              colorOf(detection.classId),
              detection.classId,
              position,
              fieldPosition,
              frame.timestampSeconds));
    }

    status = found.isEmpty() ? whyNothing : "OK";
    return found;
  }

  /**
   * Where a thing seen at these angles is, from the robot's center.
   *
   * @return the position in meters (x forward, y left), or null if the camera's line of sight
   *     never reaches the thing's height
   */
  Translation2d anglesToPosition(double txDegrees, double tyDegrees, double thingHeight) {
    // Step 1: the direction the camera looks, as the camera sees it.
    // x is straight out of the lens, y is to the left, z is up.
    double x = 1.0;
    double y = -Math.tan(Math.toRadians(txDegrees));
    double z = Math.tan(Math.toRadians(tyDegrees));

    // Step 2: tilt that direction down by the camera's tilt, like nodding your head down.
    double tilt = Math.toRadians(mountTiltDown);
    double tiltedX = x * Math.cos(tilt) + z * Math.sin(tilt);
    double tiltedZ = -x * Math.sin(tilt) + z * Math.cos(tilt);

    // Step 3: follow that direction until it reaches the thing's height.
    // If it points the wrong way, or almost level, it never gets there (or not accurately).
    double heightChange = thingHeight - mountHeight;
    double steps = heightChange / tiltedZ;
    double slope = Math.abs(tiltedZ) / Math.hypot(tiltedX, y);
    if (steps <= 0 || slope < 0.01) {
      return null;
    }

    // Step 4: turn it the way the camera faces, then move it to where the camera is
    Translation2d fromCamera = new Translation2d(steps * tiltedX, steps * y);
    fromCamera = fromCamera.rotateBy(Rotation2d.fromDegrees(mountFacing));
    return new Translation2d(mountForward, mountLeft).plus(fromCamera);
  }

  /**
   * @return ROBOT, GAME_PIECE, or null for a class we weren't told about. A class listed as both
   *     counts as a robot.
   */
  private Sighting.Type typeOf(int classId) {
    if (robotClasses.contains(classId)
        || redRobotClasses.contains(classId)
        || blueRobotClasses.contains(classId)) {
      return Sighting.Type.ROBOT;
    }
    if (gamePieceClasses.contains(classId)) {
      return Sighting.Type.GAME_PIECE;
    }
    return null;
  }

  private Sighting.Color colorOf(int classId) {
    if (redRobotClasses.contains(classId)) {
      return Sighting.Color.RED;
    }
    if (blueRobotClasses.contains(classId)) {
      return Sighting.Color.BLUE;
    }
    return Sighting.Color.UNKNOWN;
  }

  /** @return where the robot was when a picture was taken, or null without usePose(...) */
  private Pose2d poseWhenTaken(double timestampSeconds) {
    if (robotPose == null) {
      return null;
    }
    Optional<Pose2d> then = poseHistory.getSample(timestampSeconds);
    return then.orElse(robotPose.get());
  }

  /** @return the robot's pose now, or null without usePose(...) */
  Pose2d getRobotPoseOrNull() {
    return robotPose == null ? null : robotPose.get();
  }

  private List<Sighting> getAll(Sighting.Type type) {
    update();
    List<Sighting> result = new ArrayList<>();
    for (Sighting sighting : sightings) {
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

  private static void addAll(Set<Integer> set, int[] classIds) {
    for (int id : classIds) {
      set.add(id);
    }
  }

  /**
   * The pretend camera made by {@link #simulated}. It sees everything on the field (up to
   * maxDistance) from the robot's center.
   */
  static class SimulatedCamera extends Camera {
    private static final int ROBOT_CLASS = 0;
    private static final int GAME_PIECE_CLASS = 1;

    private final Supplier<List<Translation2d>> robots;
    private final Supplier<List<Translation2d>> gamePieces;

    SimulatedCamera(
        Supplier<List<Translation2d>> robots, Supplier<List<Translation2d>> gamePieces) {
      this.robots = robots;
      this.gamePieces = gamePieces;
      mountedAt(0, 0, 0, 0, 0);
      robotClasses(ROBOT_CLASS);
      gamePieceClasses(GAME_PIECE_CLASS);
    }

    @Override
    Frame readFrame() {
      Frame frame = new Frame(now());
      Pose2d pose = getRobotPoseOrNull();
      if (pose == null) {
        frame.problem = "Needs the robot's pose: call usePose(...)";
        return frame;
      }
      addThingsOnField(frame, robots.get(), ROBOT_CLASS, pose);
      addThingsOnField(frame, gamePieces.get(), GAME_PIECE_CLASS, pose);
      return frame;
    }

    private static void addThingsOnField(
        Frame frame, List<Translation2d> onField, int classId, Pose2d pose) {
      if (onField == null) {
        return;
      }
      for (Translation2d fieldPosition : onField) {
        // From the field to the robot's center: the opposite of placing it on the field
        Translation2d fromRobot =
            fieldPosition.minus(pose.getTranslation()).rotateBy(pose.getRotation().unaryMinus());
        frame.detections.add(new Detection(classId, fromRobot));
      }
    }
  }
}
