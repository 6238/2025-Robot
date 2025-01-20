package frc.robot.Subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Vision;
import frc.robot.util.Camera;
import frc.robot.util.CameraSettings;

public class VisionSubsystem extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout;

    // Keep track of cameras and poses
    private ArrayList<Camera> cameras = new ArrayList<>();
    private ArrayList<Optional<EstimatedRobotPose>> poses = new ArrayList<>();

    private SwerveSubsystem swerve;

    /** Creates a new VisionSubsystem. */
    public VisionSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
            System.out.println("Loaded apriltag field from: "+ fieldLayout);
        } catch (IOException e) {
            DataLogManager.log("ERROR: Unable to load apriltag field layout"+e.getStackTrace().toString());
        }

        cameras = new ArrayList<Camera>(Vision.CAMERA_SETTINGS.length);
        
        for (CameraSettings cameraSettings : Vision.CAMERA_SETTINGS) {
            cameras.add(new Camera(cameraSettings, fieldLayout));
            poses.add(Optional.empty());
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < cameras.size(); i++) {
            Optional<EstimatedRobotPose> pose = cameras.get(i).update();
            poses.set(i, pose);
            
            if (pose.isPresent()) {
                swerve.addVisionPose(pose.get(), Vision.VISION_STDDEV);
            }
        }
    }

    public Trigger hasPose() {
        return new Trigger(() -> {
            for (Optional<EstimatedRobotPose> pose : poses) {
                return pose.isPresent();
            }
            return false;
        });
    }
}
