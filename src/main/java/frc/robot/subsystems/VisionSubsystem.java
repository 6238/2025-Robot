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
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

@Logged
public class VisionSubsystem extends SubsystemBase {
  private AprilTagFieldLayout fieldLayout;

  // Keep track of cameras and poses
  private ArrayList<Camera> cameras = new ArrayList<>();

  public PhotonCamera algaeCam;

  @NotLogged private SwerveSubsystem swerve;

  private int count = 0;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;

    algaeCam = new PhotonCamera(Vision.ALGAECAM_NAME);

    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
      System.out.println("Loaded apriltag field from: " + fieldLayout);
    } catch (IOException e) {
      DataLogManager.log(
          "ERROR: Unable to load apriltag field layout" + e.getStackTrace().toString());
    }

    cameras = new ArrayList<Camera>(Vision.CAMERA_SETTINGS.length);

    for (CameraSettings cameraSettings : Vision.CAMERA_SETTINGS) {
      cameras.add(new Camera(cameraSettings, fieldLayout));
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.size(); i++) {
      Optional<EstimatedRobotPose> pose = cameras.get(i).update();

      if (!pose.isPresent()) {
        continue;
      }

      if (!Vision.USE_VISION) {
        return;
      }

      if (Vision.USE_ODOM_CUTOFF
          && pose.get()
                  .estimatedPose
                  .getTranslation()
                  .toTranslation2d()
                  .getDistance(swerve.getPose().getTranslation())
              > Vision.ODOM_DIST_CUTOFF) {
        continue;
      }

      if (cameras.get(i).ambiguity < 0.3) {
        swerve.addVisionPose(
            pose.get(),
            Vision.VISION_STDDEV.plus(
                Vision.INCREMENT_STDDEV.times(Math.pow(cameras.get(i).area, 2))));
        count += 1;
      }
    }
  }
}
