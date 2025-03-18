package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.SwerveSubsystem;

public class Camera {
  CameraSettings settings;
  @NotLogged
  SwerveSubsystem swerve;

  PhotonCamera camera;
  PhotonPoseEstimator estimator;

  private boolean enabled;

  public Camera(CameraSettings settings, AprilTagFieldLayout layout, SwerveSubsystem swerve) {
    this.settings = settings;
    this.swerve = swerve;

    enabled = true;

    camera = new PhotonCamera(settings.getCameraName());
    estimator = new PhotonPoseEstimator(
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

      Optional<MultiTargetPNPResult> multiTagResult = result.getMultiTagResult();

      // Calculate Ambiguity of pose
      double ambiguity = 1.0;
      if (multiTagResult.isPresent()) {
        ambiguity = multiTagResult.get().estimatedPose.ambiguity; // get multitag ambiguity
      } else {
        ambiguity = result.getBestTarget().poseAmbiguity; // get best tag ambiguity
      }

      // Calculate Average Target Area
      double avgTargetArea = 0.0;
      for (PhotonTrackedTarget target : result.targets) {
        avgTargetArea += target.area;
      }
      avgTargetArea /= result.targets.size();

      // Calculate Distance from current Position
      Optional<Double> distFromCurrentPosition = Optional.empty();
      Optional<Pose2d> poseAtResultTime = swerve.samplePoseAt(pose.get().timestampSeconds);
      if (poseAtResultTime.isPresent()) {
        Translation2d translationAtResultTime = poseAtResultTime.get().getTranslation();
        Translation2d resultTranslation = pose.get().estimatedPose.getTranslation().toTranslation2d();
        distFromCurrentPosition = Optional.of(translationAtResultTime.getDistance(resultTranslation));
      }

      Matrix<N3, N1> stdDvs = calculateStandardDevs(ambiguity, avgTargetArea, distFromCurrentPosition);
      swerve.addVisionPose(pose.get(), stdDvs);
    }
  }

  private Matrix<N3, N1> calculateStandardDevs(double ambiguity, double avgTargetArea,
      Optional<Double> distFromCurrentPosition) {
    return Vision.VISION_STDDEV;
  }
}
