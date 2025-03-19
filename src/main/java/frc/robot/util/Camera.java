package frc.robot.util;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    estimator = new PhotonPoseEstimator(
        layout, Vision.VISION_POSE_STRATEGY, settings.getCameraToRobotTransform());
  }

  @NotLogged public CameraSettings getCameraSettings() {
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

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    double[] poseArray = new double[results.size()*3];
    int i = 0;
    for (PhotonPipelineResult result : results) {
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

      SmartDashboard.putNumber("CAMERA_"+settings.getCameraName()+"AMBIGUITY", ambiguity);

      if (ambiguity > 0.05) {
        continue;
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

      poseArray[i] = pose.get().estimatedPose.getMeasureX().magnitude();
      poseArray[i+1] = pose.get().estimatedPose.getMeasureY().magnitude();
      poseArray[i+2] = pose.get().estimatedPose.getRotation().toRotation2d().getDegrees();
      i += 3;
    }

    SmartDashboard.putNumberArray("CAMERA_"+settings.getCameraName(), poseArray);
    
  }

  private Matrix<N3, N1> calculateStandardDevs(double ambiguity, double avgTargetArea,
      Optional<Double> distFromCurrentPosition) {
    return Vision.VISION_STDDEV;
  }
}
