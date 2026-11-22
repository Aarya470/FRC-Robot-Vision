package com.mercury1089.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * A PhotonVision object detection camera. Made with {@link Camera#photonVision}. It lives in its
 * own class so teams without PhotonLib can still use every other camera.
 *
 * <p>If the camera is calibrated in PhotonVision, the bottom middle of each box (where the thing
 * touches the floor) is placed using the lens calibration, which is the most accurate. If not, the
 * angles to the middle of the box are used, like a Limelight.
 */
class PhotonVisionCamera extends Camera {
  private final PhotonCamera camera;

  // The lens calibration: focal lengths and center pixel. Read with every frame, because it
  // changes if the camera is recalibrated or switched to another resolution.
  private boolean haveCalibration = false;
  private double fx;
  private double fy;
  private double cx;
  private double cy;

  PhotonVisionCamera(String name) {
    camera = new PhotonCamera(name);
  }

  @Override
  protected CameraFrame readFrame() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      if (!camera.isConnected()) {
        return CameraFrame.problem(
            now(),
            "No data from "
                + camera.getName()
                + " (check the name, and that PhotonLib matches PhotonVision's version)");
      }
      return null; // no new frame
    }

    PhotonPipelineResult newest = results.get(results.size() - 1);
    CameraFrame frame = new CameraFrame(newest.getTimestampSeconds());
    loadCalibration();

    for (PhotonTrackedTarget target : newest.getTargets()) {
      int classId = target.getDetectedObjectClassID();

      double confidence = target.getDetectedObjectConfidence();
      if (confidence >= 0 && confidence < getMinConfidence()) {
        frame.setNote("Low confidence " + String.format("%.2f", confidence));
        continue;
      }

      // Best: place the bottom of the box on the floor with the lens calibration
      if (haveCalibration && getMount() != null) {
        Translation2d onFloor = bottomOfBoxOnFloor(target);
        if (onFloor != null) {
          frame.addFoundTarget(new FoundTarget(classId, null, onFloor));
        } else {
          frame.setNote("Box bottom is above the horizon (check the camera's height and tilt)");
        }
        continue;
      }

      // Otherwise: use the angles to the middle of the box, like a Limelight
      double tx = target.getYaw(); // PhotonVision's yaw is positive to the right, like tx
      double ty = CameraMath.photonPitchToTy(target.getYaw(), target.getPitch());
      frame.addAngleTarget(new CameraTarget(classId, tx, ty));
    }
    return frame;
  }

  /** Where the bottom middle of the target's box touches the floor, or null. */
  private Translation2d bottomOfBoxOnFloor(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners == null || corners.isEmpty()) {
      corners = target.getDetectedCorners();
    }
    if (corners == null || corners.isEmpty()) {
      return null;
    }

    double left = corners.get(0).x;
    double right = corners.get(0).x;
    double bottom = corners.get(0).y;
    for (TargetCorner corner : corners) {
      left = Math.min(left, corner.x);
      right = Math.max(right, corner.x);
      bottom = Math.max(bottom, corner.y); // bigger y is lower in the image
    }
    double middle = (left + right) / 2.0;
    return CameraMath.floorPixelToRobotRelative(getMount(), fx, fy, cx, cy, middle, bottom);
  }

  /** Read the lens calibration from PhotonVision, if it has one. */
  private void loadCalibration() {
    Optional<Matrix<N3, N3>> matrix = camera.getCameraMatrix();
    if (matrix.isEmpty()) {
      haveCalibration = false;
      return;
    }
    fx = matrix.get().get(0, 0);
    fy = matrix.get().get(1, 1);
    cx = matrix.get().get(0, 2);
    cy = matrix.get().get(1, 2);
    haveCalibration = true;
  }
}
