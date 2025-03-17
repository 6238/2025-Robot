package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.util.Camera;
import frc.robot.util.CameraSettings;
import java.io.IOException;
import java.util.List;

@Logged
public class VisionSubsystem extends SubsystemBase {
  @NotLogged private AprilTagFieldLayout fieldLayout;
  private boolean failedLoadingLayout = false;

  @NotLogged private List<Camera> cameras;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerve) {
    loadFieldLayout();

    for (CameraSettings cameraSettings : Vision.CAMERA_SETTINGS) {
      cameras.add(new Camera(cameraSettings, fieldLayout, swerve));
    }
  }

  private void loadFieldLayout() {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
      System.out.println("Loaded apriltag field from: " + fieldLayout);
    } catch (IOException e) {
      failedLoadingLayout = true;
      DataLogManager.log("ERROR: Unable to load apriltag field layout" + e.toString());
    }
  }

  @Override
  public void periodic() {
    if (failedLoadingLayout) {
      return;
    }

    for (Camera camera : cameras) {
      camera.update();
    }
  }

  /* Camera Enable Functions */

  public void enableCamera(String cameraName) {
    for (Camera camera : cameras) {
      if (camera.getCameraSettings().getCameraName().equals(cameraName)) {
        camera.setEnabled(true);
      }
    }
  }

  public void disableCamera(String cameraName) {
    for (Camera camera : cameras) {
      if (camera.getCameraSettings().getCameraName().equals(cameraName)) {
        camera.setEnabled(false);
      }
    }
  }

  public void disableAllCameras() {
    for (Camera camera : cameras) {
      camera.setEnabled(false);
    }
  }

  public void enableOnlyOneCamera(String cameraName) {
    disableAllCameras();
    enableCamera(cameraName);
  }
}
