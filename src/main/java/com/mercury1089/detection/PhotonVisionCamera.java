package com.mercury1089.detection;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A PhotonVision camera running an object detection pipeline. Made with {@link
 * Camera#photonVision}. It is in its own file so teams without PhotonLib can still use the rest.
 */
class PhotonVisionCamera extends Camera {
  private final PhotonCamera camera;

  PhotonVisionCamera(String name) {
    camera = new PhotonCamera(name);
  }

  @Override
  Frame readFrame() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      if (!camera.isConnected()) {
        Frame frame = new Frame(now());
        frame.problem =
            "No data from "
                + camera.getName()
                + " (check the name, and that PhotonLib matches PhotonVision's version)";
        return frame;
      }
      return null; // no new picture
    }

    PhotonPipelineResult newest = results.get(results.size() - 1);
    Frame frame = new Frame(newest.getTimestampSeconds());
    for (PhotonTrackedTarget target : newest.getTargets()) {
      frame.detections.add(
          new Detection(
              target.getDetectedObjectClassID(),
              target.getYaw(),
              pitchToTy(target.getYaw(), target.getPitch())));
    }
    return frame;
  }

  /**
   * PhotonVision's yaw is the same as Limelight's tx (positive = right). Its pitch is measured a
   * little differently from Limelight's ty, so this turns it into a ty.
   */
  static double pitchToTy(double yawDegrees, double pitchDegrees) {
    double tanPitch = Math.tan(Math.toRadians(pitchDegrees));
    double cosYaw = Math.cos(Math.toRadians(yawDegrees));
    return Math.toDegrees(Math.atan(tanPitch / cosYaw));
  }
}
