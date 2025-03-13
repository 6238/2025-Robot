package frc.robot.commands;

import static frc.robot.Constants.Swerve.MAX_SPEED;

import edu.wpi.first.epilogue.Logged;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class AimAtAlgae extends TurnToAngle {
  public VisionSubsystem vision;
  public SwerveSubsystem swerve;
  public static double targetYaw;
  public static double swerve_x;
  public static double swerve_y;
  boolean targetVisible = false;

  public AimAtAlgae(VisionSubsystem vision, SwerveSubsystem swerve) {
    super(swerve, () -> targetYaw, () -> swerve_x, () -> swerve_y, true);
    this.vision = vision;
    this.swerve = swerve;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    targetVisible = false;
    List<PhotonPipelineResult> results = vision.algaeCam.getAllUnreadResults();

    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        targets.removeIf(
            (photonTrackedTarget) -> {
              return photonTrackedTarget.pitch > 0;
            });

        if (targets.size() == 0) {
          return;
        }

        PhotonTrackedTarget bestTarget = targets.get(0);
        double maxScore = 0;
        for (PhotonTrackedTarget target : targets) {
          double targetScore = target.area;
          if (targetScore > maxScore) {
            bestTarget = target;
          }
        }

        targetYaw = swerve.getPose().getRotation().getDegrees() - bestTarget.getYaw();
        swerve_x = MAX_SPEED / 2;
        targetVisible = true;
      }
    }

    super.execute();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
