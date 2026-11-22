package com.mercury1089.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;

/**
 * A Limelight running a neural detector pipeline. Made with {@link Camera#limelight}.
 *
 * <p>It reads what the Limelight publishes on NetworkTables:
 *
 * <ul>
 *   <li>"rawdetections": 12 numbers per detection. The first three are the class number and the
 *       angles to the middle of the box (tx and ty, measured from the image's center).
 *   <li>"hb": a heartbeat that goes up by one every frame
 *   <li>"tl" and "cl": how long the picture took, in milliseconds
 * </ul>
 */
class LimelightCamera extends Camera {
  /** Numbers per detection in "rawdetections" */
  static final int NUMBERS_PER_DETECTION = 12;

  /** Seconds without a new frame before the Limelight counts as disconnected */
  static final double NO_DATA_AFTER_SECONDS = 0.5;

  private final String name;
  private final DoubleArraySubscriber rawDetections;
  private final DoubleSubscriber pipelineLatencyMs;
  private final DoubleSubscriber captureLatencyMs;
  private final DoubleSubscriber heartbeat;

  private double lastHeartbeat = -1;
  private double lastNewFrameTime = -1;

  private LimelightCamera(String name, NetworkTable table) {
    this.name = name;
    rawDetections = table.getDoubleArrayTopic("rawdetections").subscribe(new double[0]);
    pipelineLatencyMs = table.getDoubleTopic("tl").subscribe(0.0);
    captureLatencyMs = table.getDoubleTopic("cl").subscribe(0.0);
    heartbeat = table.getDoubleTopic("hb").subscribe(-1.0);
  }

  static LimelightCamera create(String name) {
    if (name == null || name.isBlank()) {
      name = "limelight";
    }
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return new LimelightCamera(name, table);
  }

  @Override
  protected CameraFrame readFrame() {
    double now = now();
    double beat = heartbeat.get();

    if (beat < 0) {
      return CameraFrame.problem(now, "No data from " + name + " (check the name and that it is on)");
    }
    if (beat == lastHeartbeat) {
      // No new frame. After a while with none, say so.
      if (now - lastNewFrameTime > NO_DATA_AFTER_SECONDS) {
        return CameraFrame.problem(now, "No new frames from " + name);
      }
      return null;
    }
    lastHeartbeat = beat;
    lastNewFrameTime = now;

    double latencySeconds = (pipelineLatencyMs.get() + captureLatencyMs.get()) / 1000.0;
    CameraFrame frame = new CameraFrame(now - latencySeconds);
    for (CameraTarget target : parse(rawDetections.get())) {
      frame.addAngleTarget(target);
    }
    return frame;
  }

  /** Turn Limelight's "rawdetections" numbers into targets. */
  static List<CameraTarget> parse(double[] raw) {
    List<CameraTarget> targets = new ArrayList<>();
    if (raw == null) {
      return targets;
    }
    int count = raw.length / NUMBERS_PER_DETECTION;
    for (int i = 0; i < count; i++) {
      int start = i * NUMBERS_PER_DETECTION;
      int classId = (int) raw[start];
      double tx = raw[start + 1];
      double ty = raw[start + 2];
      targets.add(new CameraTarget(classId, tx, ty));
    }
    return targets;
  }
}
