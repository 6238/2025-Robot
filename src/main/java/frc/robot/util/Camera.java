package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.Constants.Vision;
import frc.robot.telemetry.Alert;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Utility class to work with reading from a camera attached via PhotonVision and feeding the data
 * into a {@link org.photonvision.PhotonPoseEstimator}.
 */
public class Camera {
  private PhotonCamera cam;
  private PhotonPoseEstimator poseEst;

  private Alert camDisconnected;
  public double ambiguity;

  /**
   * Creates a new Camera.
   *
   * @param camName The name of the camera - this must match what's set in the PhotonVision
   *     settings.
   * @param robotToCam An {@link edu.wpi.first.math.geometry.Transform3d} representing the distance
   *     and rotation from the robot's center to the camera's lens.
   * @param layout The layout of the field.
   */
  public Camera(CameraSettings settings, AprilTagFieldLayout layout) {
    cam = new PhotonCamera(settings.getCameraName());
    poseEst =
        new PhotonPoseEstimator(
            layout, Vision.VISION_POSE_STRATEGY, settings.getCameraToRobotTransform());

    camDisconnected =
        new Alert(
            "Camera " + settings.getCameraName() + " is disconnected!", Alert.AlertType.ERROR);
  }

  public Optional<EstimatedRobotPose> update() {
    if (!cam.isConnected()) {
      camDisconnected.set(true);
      return Optional.empty();
    }
    camDisconnected.set(false);

    Optional<EstimatedRobotPose> pose = Optional.empty();
    for (PhotonPipelineResult result : cam.getAllUnreadResults()) {
      ambiguity = 0.0;

      for (PhotonTrackedTarget target : result.getTargets()) {
        ambiguity += target.poseAmbiguity;
      }

      pose = poseEst.update(result, cam.getCameraMatrix(), cam.getDistCoeffs());
    }

    return pose;
  }
}
