package com.mercury1089.vision;

import java.util.List;
import java.util.function.Supplier;

/** Any camera that gives angles to what it sees. Made with {@link Camera#fromAngles}. */
class AnglesCamera extends Camera {
  private final Supplier<List<CameraTarget>> targets;

  AnglesCamera(Supplier<List<CameraTarget>> targets) {
    this.targets = targets;
  }

  @Override
  protected CameraFrame readFrame() {
    CameraFrame frame = new CameraFrame(now());
    List<CameraTarget> list = targets.get();
    if (list != null) {
      for (CameraTarget target : list) {
        frame.addAngleTarget(target);
      }
    }
    return frame;
  }
}
