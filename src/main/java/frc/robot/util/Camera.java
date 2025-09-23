package frc.robot.util;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.SwerveSubsystem;

@Logged
public class Camera {
  @NotLogged CameraSettings settings;
  @NotLogged SwerveSubsystem swerve;

  @NotLogged PhotonCamera camera;
  @NotLogged PhotonPoseEstimator estimator;

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

  @NotLogged
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
    estimator.setLastPose(swerve.getPose());

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (results.isEmpty()) {
      return;
    }

    for (PhotonPipelineResult result : results) {
      Optional<EstimatedRobotPose> pose = estimator.update(result);
      if (pose.isEmpty()) {
        continue;
      }

      // Calculate Ambiguity of pose
      double ambiguity = result.getBestTarget().poseAmbiguity;
      if (ambiguity > Vision.AMBIGUITY_CUTOFF && Vision.AMBIGUITY_CUTOFF_ENABLE) {
        continue;
      }

      // Calculate Average Target Area
      double distanceToTag =
          result
              .getBestTarget()
              .bestCameraToTarget
              .getTranslation()
              .getDistance(Translation3d.kZero);
      if (distanceToTag > Vision.CLOSE_FAR_CUTOFF && Vision.CLOSE_FAR_CUTOFF_ENABLE) {
        continue;
      }

      Matrix<N3, N1> stdDvs = calculateStandardDevs(ambiguity, distanceToTag);
      swerve.addVisionPose(pose.get(), stdDvs);
    }
  }

  private Matrix<N3, N1> calculateStandardDevs(double ambiguity, double targetDistance) {
    if (targetDistance > Vision.CLOSE_FAR_CUTOFF) {
      return Vision.REEF_FAR_VISION_STDDEV;
    }
    return Vision.REEF_CLOSE_VISION_STDDEV;
  }
}
