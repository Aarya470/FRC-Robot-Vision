# Robot Detection

See **robots** and **game pieces** with a Limelight or PhotonVision camera, and find out where they are. Made by Aaryan Roy on FRC Team 1089 Mercury.

It works on its own: you don't need PathPlanner or any other library.

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
https://aarya470.github.io/FRC-Robot-Vision/RobotDetection.json
```

For PhotonVision cameras, also install the [PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html) vendor library (its version must match PhotonVision on your coprocessor). Limelights need nothing extra.

## Set up a camera

Make one `Camera` per camera, once, for example as a field in `RobotContainer`:

```java
import com.mercury1089.detection.Camera;
import com.mercury1089.detection.Sighting;

Camera frontCamera = Camera.limelight("limelight-front")   // or Camera.photonVision("CameraName")
    .mountedAt(0.30, 0.0, 0.75, 15, 0)
    .redRobotClasses(1)
    .blueRobotClasses(2)
    .gamePieceClasses(0)
    .usePose(drivetrain::getPose);   // optional
```

**`mountedAt(forward, left, height, tiltDown, facing)`**, measured from the robot's center on the floor:

- **forward**, **left**: meters from the center (negative = behind, right)
- **height**: meters from the floor up to the lens
- **tiltDown**: degrees tilted down (0 = level)
- **facing**: 0 = forward, 90 = left, 180 = backward, -90 = right

Mount the camera well above (or below) the middle of the things it looks for. A camera at the same height looks straight across at them, so it can't tell how far away they are. `tools/camera_check.py` checks your numbers with one tape measurement (`python tools/camera_check.py --help`).

**Classes.** Your detection model gives each thing a class number. Tell the camera which is which: `redRobotClasses(...)` and `blueRobotClasses(...)` if the model tells alliances apart, `robotClasses(...)` if it doesn't, and `gamePieceClasses(...)`. Classes you don't list are ignored. With no classes at all, it sees nothing.

**Optional:**

- `usePose(drivetrain::getPose)`: sightings also get field positions
- `robotHeight(meters)` (default 0.4) and `gamePieceHeight(meters)` (default 0.08): how high the middle of each is
- `maxDistance(meters)` (default 8): ignore anything farther

## Ask what it sees

| Method | Gives |
| --- | --- |
| `seesSomething()`, `seesRobot()`, `seesGamePiece()` | `true` or `false` |
| `seesRobotWithin(meters)`, `seesGamePieceWithin(meters)` | `true` if one is that close |
| `getClosestRobot()`, `getClosestGamePiece()` | a `Sighting`, or `null` if there is none |
| `getRobots()`, `getGamePieces()` | a list of `Sighting`s |
| `getStatus()` | `"OK"`, or why it sees nothing |

A `Sighting` has `getDistanceMeters()`, `getAngleDegrees()` (0 ahead, 90 left), `getRobotRelativePosition()` (x forward, y left), `getFieldPosition()` (`null` without `usePose`), `isRobot()`, `isGamePiece()`, `getColor()` (`RED`, `BLUE` or `UNKNOWN`) and `getClassId()`.

## Commands and triggers

```java
frontCamera.robotNearby(1.5).onTrue(Commands.print("Robot close!"));
intakeCamera.gamePieceSeen().whileTrue(leds.showGreen());

Command waitForPiece = intakeCamera.waitUntilSeesGamePiece();
Command waitForClearPath = frontCamera.waitUntilNoRobotWithin(2.0);

Command getPiece = new DriveToGamePieceCommand(
        intakeCamera,
        drivetrain::driveRobotRelative,   // your method that takes robot-relative ChassisSpeeds
        2.0,                              // top speed, meters per second
        true,                             // true if the intake is on the back
        0.5,                              // stop when the piece is this close to the robot's center
        drivetrain)                       // your drivetrain subsystem
    .deadlineFor(yourIntakeCommand);      // runs your intake until it gets there
```

`DriveToGamePieceCommand` drives forward and turns, so it works on any drivetrain. It sticks with the closest piece and stops when it gets there. If the piece goes under the bumper (out of view) while close and straight ahead, it creeps the rest of the way. If something could block the robot, add `.withTimeout(3)`.

## Simulation

`Camera.simulated(() -> robotPositions, () -> gamePiecePositions)` makes a pretend camera that sees robots and game pieces where you say they are on the field. Give it `usePose(...)`.

## Dashboard and problems

```java
SmartDashboard.putData("Front camera", frontCamera);
```

This shows the status and how many robots and game pieces it sees. If the status isn't `OK`, it says why:

| Status | What to do |
| --- | --- |
| `Sees nothing` | Working, but nothing is in view |
| `No data from limelight-front ...` | Check the name, and that the camera is on. For PhotonVision, also that PhotonLib's version matches |
| `No new pictures from limelight-front` | The Limelight stopped sending pictures. Check it is running a neural detector pipeline |
| `No classes set: ...` | Add `robotClasses(...)` or `gamePieceClasses(...)` |
| `Not mounted: call mountedAt(...)` | Add `mountedAt(...)` |
| `Ignored class 3` | The model sees class 3 but you didn't list it |
| `Can't place what it sees: ...` | Check the height and tilt in `mountedAt(...)` |
| `Farther than maxDistance` | It only sees things farther than `maxDistance(...)` |

## Making a release (maintainers)

Add a `## X.Y.Z` section to `CHANGELOG.md`, then push a tag like `vX.Y.Z`. The release workflow builds the library and puts it, with `RobotDetection.json`, on GitHub Pages.

## Use it with PathPlanner

[PathPlanner Backup Paths](https://github.com/Aarya470/pathplanner-backup-reactive-autos-paths) can use these cameras to wait, divert or pick up game pieces in the middle of an auto.

## License

MIT. See `LICENSE`.
