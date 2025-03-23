package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAtAlgae extends TurnToAngle {
    private static double swerveX = 0.0;
    private static double swerveY = 0.0;
    private static double targetYaw = 0.0;
    private boolean targetVisible = false;

    VisionSubsystem vision;

    public AimAtAlgae(VisionSubsystem vision, SwerveSubsystem swerve) {
        super(swerve, () -> targetYaw, () -> swerveX, () -> swerveY, true);
        this.vision = vision;
    }

    @Override
  public void initialize() {}

  @Override
  public void execute() {
    targetVisible = false;
    List<PhotonPipelineResult> results = vision.getAlgaeCamResults();

    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        targets.removeIf(
            (photonTrackedTarget) -> {
              return photonTrackedTarget.pitch > 0;
            });

        if (targets.size() == 0) {
          super.execute();
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
        swerveX = Constants.Swerve.MAX_SPEED;
        targetVisible = true;
      }
    }

    super.execute();
    }

    @Override
    public void end(boolean interrupted) {
      targetVisible = false;
      swerveX = 0.0;
      swerveY = 0.0;
      targetYaw = swerve.getPose().getRotation().getDegrees();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
