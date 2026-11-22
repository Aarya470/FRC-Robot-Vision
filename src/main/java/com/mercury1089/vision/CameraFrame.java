package com.mercury1089.vision;

import java.util.ArrayList;
import java.util.List;

/**
 * Everything a camera saw in one picture. Targets come either as angles (the Camera class works
 * out where they are) or as positions the camera already worked out.
 */
public class CameraFrame {
  private final List<CameraTarget> angleTargets = new ArrayList<>();
  private final List<FoundTarget> foundTargets = new ArrayList<>();
  private final double timestampSeconds;
  private String problem = null;
  private String note = null;

  /** @param timestampSeconds when the picture was taken (the same clock as Timer.getFPGATimestamp()) */
  public CameraFrame(double timestampSeconds) {
    this.timestampSeconds = timestampSeconds;
  }

  /** A frame that says why the camera has no data, such as "No data from limelight-front". */
  public static CameraFrame problem(double timestampSeconds, String problem) {
    CameraFrame frame = new CameraFrame(timestampSeconds);
    frame.problem = problem;
    return frame;
  }

  /** Add a target seen as angles. */
  public void addAngleTarget(CameraTarget target) {
    angleTargets.add(target);
  }

  /** Add a target whose position is already known. */
  public void addFoundTarget(FoundTarget target) {
    foundTargets.add(target);
  }

  /** Explain why a target was dropped, shown when nothing else gets through. */
  public void setNote(String note) {
    this.note = note;
  }

  public List<CameraTarget> getAngleTargets() {
    return angleTargets;
  }

  public List<FoundTarget> getFoundTargets() {
    return foundTargets;
  }

  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  public String getProblem() {
    return problem;
  }

  public String getNote() {
    return note;
  }
}
