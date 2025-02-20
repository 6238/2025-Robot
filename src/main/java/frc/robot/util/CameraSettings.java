package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraSettings {
  private String cameraName;
  private Transform3d cameraToRobotTransform;

  public CameraSettings(String cameraName, Transform3d cameraToRobotTransform) {
    this.cameraName = cameraName;
    this.cameraToRobotTransform = cameraToRobotTransform;
  }

  public String getCameraName() {
    return cameraName;
  }

  public Transform3d getCameraToRobotTransform() {
    return cameraToRobotTransform;
  }
}
