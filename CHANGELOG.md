# Changelog

## 1.0.0

- First release. Object detection for FRC robots with a Limelight, a PhotonVision camera, or any other camera.
- `Camera.limelight(...)`, `Camera.photonVision(...)`, `Camera.fromAngles(...)` and `Camera.fromFieldPositions(...)`.
- Ask what it sees: `seesRobot()`, `seesGamePiece()`, `seesRobotWithin(meters)`, `getClosestRobot()`, `getClosestGamePiece()`, and more.
- `getStatus()` and the dashboard explain why nothing comes through.
- Ready-made commands and triggers: `VisionCommands.driveToClosestGamePiece(...)`, `waitUntilNoRobotWithin(...)`, `camera.robotNearby(meters)`, `camera.gamePieceSeen()`.
