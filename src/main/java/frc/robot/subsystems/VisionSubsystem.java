package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.util.Camera;
import java.io.IOException;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

@Logged
public class VisionSubsystem extends SubsystemBase {
  @NotLogged private AprilTagFieldLayout fieldLayout;
  private boolean failedLoadingLayout = false;

  private Camera cameras[] = new Camera[Vision.CAMERA_SETTINGS.length];

  PhotonCamera algaeCamera = new PhotonCamera("AlgaeCam");

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerve) {
    loadFieldLayout();

    for (int i = 0; i < Vision.CAMERA_SETTINGS.length; i++) {
      cameras[i] = new Camera(Vision.CAMERA_SETTINGS[i], fieldLayout, swerve);
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

    if (!Vision.ENABLE) {
      return;
    }

    for (Camera camera : cameras) {
      camera.update();
    }
  }

  public List<PhotonPipelineResult> getAlgaeCamResults() {
    return algaeCamera.getAllUnreadResults();
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
