package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.util.Camera;
import frc.robot.util.CameraSettings;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

@Logged
public class VisionSubsystem extends SubsystemBase {
  private AprilTagFieldLayout fieldLayout;

  // Keep track of cameras and poses
  private ArrayList<Camera> cameras = new ArrayList<>();

  private SwerveSubsystem swerve;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;

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

      if (pose.isPresent()) {
        swerve.addVisionPose(pose.get(), Vision.VISION_STDDEV);
      }
    }
  }
}
