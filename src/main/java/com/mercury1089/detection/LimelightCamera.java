package com.mercury1089.detection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A Limelight running a neural detector pipeline. Made with {@link Camera#limelight}.
 *
 * <p>It reads what the Limelight puts on NetworkTables:
 *
 * <ul>
 *   <li>"rawdetections": 12 numbers per thing seen. The first three are the class number and the
 *       angles to the middle of its box (tx and ty).
 *   <li>"hb": a heartbeat that goes up by one every picture
 *   <li>"tl" and "cl": how long the picture took, in milliseconds
 * </ul>
 */
class LimelightCamera extends Camera {
  private final String name;
  private final NetworkTable table;
  private double lastHeartbeat = -1;
  private double lastNewPictureTime = -1;

  LimelightCamera(String name) {
    if (name == null || name.isBlank()) {
      name = "limelight";
    }
    this.name = name;
    this.table = NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  Frame readFrame() {
    double now = now();
    double heartbeat = table.getEntry("hb").getDouble(-1);

    if (heartbeat < 0) {
      Frame frame = new Frame(now);
      frame.problem = "No data from " + name + " (check the name and that it is on)";
      return frame;
    }
    if (heartbeat == lastHeartbeat) {
      // No new picture. After half a second with none, say so.
      if (now - lastNewPictureTime > 0.5) {
        Frame frame = new Frame(now);
        frame.problem = "No new pictures from " + name;
        return frame;
      }
      return null;
    }
    lastHeartbeat = heartbeat;
    lastNewPictureTime = now;

    double latencyMs = table.getEntry("tl").getDouble(0) + table.getEntry("cl").getDouble(0);
    Frame frame = new Frame(now - latencyMs / 1000.0);
    readDetections(table.getEntry("rawdetections").getDoubleArray(new double[0]), frame);
    return frame;
  }

  /** Add each thing in Limelight's "rawdetections" numbers to the picture. */
  static void readDetections(double[] raw, Frame frame) {
    for (int start = 0; start + 12 <= raw.length; start += 12) {
      int classId = (int) raw[start];
      double tx = raw[start + 1];
      double ty = raw[start + 2];
      frame.detections.add(new Detection(classId, tx, ty));
    }
  }
}
