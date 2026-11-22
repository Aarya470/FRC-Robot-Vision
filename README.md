# Mercury Vision

Object detection for FRC robots, made by Aaryan Roy on FRC Team 1089 Mercury. Use a Limelight, a PhotonVision camera, or any other camera to see **robots** and **game pieces**, and find out where they are and how far away.

It works on its own. You don't need PathPlanner or any other library to use it.

```java
if (frontCamera.seesRobotWithin(1.5)) {
    // a robot is close
}

Sighting piece = intakeCamera.getClosestGamePiece();   // null if it sees none
if (piece != null) {
    System.out.println("Game piece " + piece.getDistanceMeters() + " m away");
}
```

## Install

In WPILib VS Code, run **WPILib: Manage Vendor Libraries > Install new libraries (online)** and paste:

```
https://aarya470.github.io/FRC-Robot-Vision/MercuryVision.json
```

For PhotonVision cameras, also install the [PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html) vendor library. Its version must match the PhotonVision version on your coprocessor. Limelight and other cameras need nothing extra.

## Set up a camera

Make one camera object per camera, once, for example as a field in `RobotContainer`:

```java
import com.mercury1089.vision.Camera;

Camera frontCamera = Camera.limelight("limelight-front")
    .mountedAt(0.30, 0.0, 0.75, 15, 0)
    .usePose(drivetrain::getPose)
    .redRobotClasses(1)
    .blueRobotClasses(2)
    .gamePieceClasses(0);
```

### 1. Pick the camera

| Camera | How to make it | Needs |
| --- | --- | --- |
| Limelight with a neural detector pipeline | `Camera.limelight("limelight-name")` | nothing extra |
| PhotonVision object detection | `Camera.photonVision("CameraName")` | PhotonLib. Calibrate the camera for the best distances. |
| Any camera that gives angles | `Camera.fromAngles(() -> myTargets)` | a list of `new CameraTarget(classId, tx, ty)`: tx = degrees right of center, ty = degrees above center |
| Anything that gives field positions | `Camera.fromFieldPositions(ThingType.ROBOT, () -> myPositions)` | `usePose(...)` |

### 2. Say where it sits: `mountedAt(forward, left, height, tiltDown, facing)`

Measure from the robot's center, on the floor:

- **forward**: meters in front of the center (negative = behind)
- **left**: meters left of the center (negative = right)
- **height**: meters from the floor up to the lens
- **tiltDown**: degrees the camera is tilted down (0 = level)
- **facing**: degrees it points: 0 = forward, 90 = left, 180 = backward, -90 = right

`tools/camera_check.py` works out the tilt from one tape measurement and checks distances. Run `python tools/camera_check.py --help`.

### 3. Tell it what the model's classes are

Your detection model gives each thing a class number. Tell the camera which is which:

- `redRobotClasses(...)` and `blueRobotClasses(...)` for robots, if your model tells the alliances apart
- `robotClasses(...)` for robots, if it doesn't
- `gamePieceClasses(...)` for game pieces

Classes you don't list are ignored. If you list none at all, everything counts, as `ThingType.OTHER`. OTHER things are never robots or game pieces, so `seesRobot()` and `seesGamePiece()` stay `false` (and Reactive Autonomous Pathing can't use the camera). OTHER things are placed using the game piece height. `hasClassesSet()` tells you whether any classes were listed.

### 4. Optional

- `usePose(drivetrain::getPose)`: sightings also get field positions. Frames arrive a little late, so each one is placed using where the robot was when the frame was taken.
- `robotHeight(meters)` (default 0.4) and `gamePieceHeight(meters)` (default 0.08): how high the middle of each is. Change these if yours are very different. Mount the camera well above (or below) these heights: a camera near a target's height looks almost straight across at it, so distances get very inaccurate. Keep at least 15 cm between them.
- `maxDistance(meters)` (default 8): ignore anything farther
- `maxAge(seconds)` (default 0.25): forget sightings older than this
- `minConfidence(0 to 1)` (default 0.5): PhotonVision only. For a Limelight, set it on the Limelight's web page.

## Ask what it sees

| Method | Gives |
| --- | --- |
| `seesSomething()`, `seesRobot()`, `seesGamePiece()` | `true` or `false` |
| `seesRobotWithin(meters)`, `seesGamePieceWithin(meters)`, `seesSomethingWithin(meters)` | `true` if one is that close |
| `getClosestRobot()`, `getClosestGamePiece()`, `getClosest()` | a `Sighting`, or `null` if there is none |
| `getRobots()`, `getGamePieces()`, `getSightings()` | a list of `Sighting`s |
| `getStatus()` | `"OK"`, or why nothing comes through |

A `Sighting` has:

- `getDistanceMeters()`: how far from the robot's center
- `getAngleDegrees()`: which way (0 ahead, 90 left, -90 right, 180 behind)
- `getRobotRelativePosition()`: x forward, y left, in meters
- `getFieldPosition()`: where on the field, or `null` without `usePose(...)`
- `getType()`, `isRobot()`, `isGamePiece()`
- `getColor()`: `RED`, `BLUE` or `UNKNOWN`
- `getClassId()`

## Commands and triggers

```java
import com.mercury1089.vision.VisionCommands;

// Triggers
frontCamera.robotNearby(1.5).onTrue(Commands.print("Robot close!"));
intakeCamera.gamePieceSeen().whileTrue(leds.showGreen());

// Commands
Command waitForPiece = VisionCommands.waitUntilSeesGamePiece(intakeCamera);
Command waitForClearPath = VisionCommands.waitUntilNoRobotWithin(frontCamera, 2.0);
Command getPiece = VisionCommands.driveToClosestGamePiece(
        intakeCamera,
        drivetrain::driveRobotRelative,   // your method that takes robot-relative ChassisSpeeds
        2.0,                              // top speed, meters per second
        true,                             // true if the intake is on the back
        0.5,                              // stop when the piece is this close to the robot's center
        drivetrain)                       // your drivetrain subsystem
    .deadlineFor(yourIntakeCommand);      // runs your intake until it gets there
```

`driveToClosestGamePiece` drives forward and turns, so it works on any drivetrain. It picks the closest piece and sticks with it. It ends when:

- the piece is within the arrive distance (about half your robot's length with bumpers), or
- the camera lost the piece when it was close (under 1.2 m) and nearly straight ahead, usually because it went under the bumper. The robot then drives slowly straight on for as long as the rest of the way should take, then stops.
- the camera lost the piece any other way for half a second.

With `usePose(...)` on the camera, it follows the piece by its field position, which keeps it on the same piece even when driving fast. If something could stop the robot reaching the piece, add a time limit with `.withTimeout(3)`.

## See it on the dashboard

```java
SmartDashboard.putData("Front camera", frontCamera);
```

This shows the status, how many robots and game pieces it sees, and the closest distances. If the status isn't `OK`, it says why, for example:

| Status | What to do |
| --- | --- |
| `OK` | Working |
| `OK (no classes set: everything is OTHER)` | It sees things, but you didn't list any classes, so nothing counts as a robot or game piece. Add `...Classes(...)` |
| `No frames yet` | It hasn't been asked yet, or the camera hasn't sent anything |
| `Sees nothing` | Working, but nothing is in view |
| `No data from limelight-front` | Check the name, and that the camera is on and connected |
| `No data from DetectionCam (check the name...)` | PhotonVision: check the name, and that PhotonLib's version matches PhotonVision's |
| `No new frames from limelight-front` | Limelight: it is connected but has stopped sending frames. Check its pipeline is a neural detector and it is running |
| `Needs the robot's pose: call usePose(...)` | `fromFieldPositions` cameras need `usePose(...)` |
| `Low confidence 0.42` | Detections were below `minConfidence(...)` |
| `Box bottom is above the horizon` | PhotonVision: check the height and tilt in `mountedAt(...)` |
| `Not mounted: call mountedAt(...)` | Add `mountedAt(...)` |
| `Ignored class 3` | The model sees class 3 but you didn't list it. Add it to the right `...Classes(...)` |
| `Can't place a target` | Check the height and tilt in `mountedAt(...)` |
| `Camera is level with the target's height` | The camera is at the same height as the middle of the targets, so distances can't be worked out. Mount it higher or lower. |

## One camera, both robots and game pieces

One camera can see both. List both kinds of classes on it, then ask `seesRobot()` or `seesGamePiece()`. Robots and game pieces each use their own height.

## Making a release (maintainers)

Push a tag like `v1.0.0` (with a `## 1.0.0` section in `CHANGELOG.md`). The release workflow builds the library and puts it, with `MercuryVision.json`, on the `gh-pages` branch.

The first time only: in the repo's **Settings > Pages**, set the source to **Deploy from a branch**, branch `gh-pages`, folder `/ (root)`. Then check that https://aarya470.github.io/FRC-Robot-Vision/MercuryVision.json opens.

## Use it with PathPlanner

[PathPlanner Backup Paths](https://github.com/Aarya470/pathplanner-backup-reactive-autos-paths) can use Mercury Vision cameras to wait, divert or pick up game pieces in the middle of an auto. Mercury Vision doesn't need it, though.

## License

MIT. See `LICENSE`.
