package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAtAlgae extends Command {
  public VisionSubsystem vision;
  public SwerveSubsystem swerve;
  boolean targetVisible = false;

  public AimAtAlgae(VisionSubsystem vision, SwerveSubsystem swerve) {
    this.vision = vision;
    this.swerve = swerve;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    targetVisible = false;
    double targetYaw = 0.0;
    double minPitch = Double.MAX_VALUE;
    List<PhotonPipelineResult> results = vision.algaeCam.getAllUnreadResults();

    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getTargets().get(0); // Lowest Algae
        if (target.pitch < minPitch) {
          targetYaw = target.getYaw();
          minPitch = target.pitch;
          targetVisible = true;
        }
      }
    }

    double turn = 0.0;

    if (targetVisible) {
      turn = -1.0 * targetYaw * Vision.VISION_TURN_kP * Constants.Swerve.MAX_ANGULAR_VELOCITY;
    }

    swerve.drive(new Translation2d(), turn, false);

    SmartDashboard.putBoolean("Algae Target Visible", targetVisible);
    SmartDashboard.putNumber("Algae Target Yaw", targetYaw);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !targetVisible;
  }
}
