# Changelog

## 2.0.0

Renamed from Mercury Vision to **Robot Detection**, and made much simpler: 5 files instead of 15.

- New install link: `https://aarya470.github.io/FRC-Robot-Vision/RobotDetection.json`. Remove `MercuryVision.json` from `vendordeps/`.
- The package is now `com.mercury1089.detection`.
- `ThingType` and `TeamColor` are now `Sighting.Type` and `Sighting.Color`.
- The wait commands are on the camera: `camera.waitUntilSeesGamePiece()`, `camera.waitUntilSeesRobot()`, `camera.waitUntilNoRobotWithin(meters)`.
- `VisionCommands.driveToClosestGamePiece(...)` is now `new DriveToGamePieceCommand(...)`, with the same arguments.
- `Camera.fromFieldPositions(...)` is now `Camera.simulated(robots, gamePieces)`.
- Removed: `Camera.fromAngles(...)`, the `OTHER` type (a camera with no classes now sees nothing), `minConfidence`, `maxAge`, `getSightings()`, `getClosest()` and `seesSomethingWithin()`. Set confidence on the Limelight or PhotonVision page instead.
- PhotonVision cameras use the angles to each box's middle, the same as Limelights.
- The dashboard shows the status and how many robots and game pieces it sees. The closest-distance entries are gone.
- `tools/camera_check.py` only takes `--tx` and `--ty` now (no `--lens` or `--pixel`).

## 1.0.0

- First release. Object detection for FRC robots with a Limelight, a PhotonVision camera, or any other camera.
- `Camera.limelight(...)`, `Camera.photonVision(...)`, `Camera.fromAngles(...)` and `Camera.fromFieldPositions(...)`.
- Ask what it sees: `seesRobot()`, `seesGamePiece()`, `seesRobotWithin(meters)`, `getClosestRobot()`, `getClosestGamePiece()`, and more.
- `getStatus()` and the dashboard explain why nothing comes through.
- Ready-made commands and triggers: `VisionCommands.driveToClosestGamePiece(...)`, `waitUntilNoRobotWithin(...)`, `camera.robotNearby(meters)`, `camera.gamePieceSeen()`.
