package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.NotLogged;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera {
  CameraSettings settings;
  @NotLogged SwerveSubsystem swerve;

  PhotonCamera camera;
  PhotonPoseEstimator estimator;

  private boolean enabled;

  public Camera(CameraSettings settings, AprilTagFieldLayout layout, SwerveSubsystem swerve) {
    this.settings = settings;
    this.swerve = swerve;

    enabled = true;

    camera = new PhotonCamera(settings.getCameraName());
    estimator =
        new PhotonPoseEstimator(
            layout, Vision.VISION_POSE_STRATEGY, settings.getCameraToRobotTransform());
  }

  public CameraSettings getCameraSettings() {
    return settings;
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public boolean getEnabled() {
    return enabled;
  }

  public void update() {
    if (!enabled) {
      return;
    }

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> pose = estimator.update(result);

      if (pose.isEmpty()) {
        continue;
      }

      swerve.addVisionPose(pose.get());
    }
  }
}
