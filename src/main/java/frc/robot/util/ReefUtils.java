package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Elevator.ElevatorHeights;
import frc.robot.Constants.PathfindingConfig;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.AlgaeEndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ReefUtils {

  /*
   * Returns angle to reef in degrees
   */
  public static double AngleToReef(Pose2d pose, Pose2d reefPose) {
    Transform2d diff = pose.minus(reefPose);
    double angle = Units.radiansToDegrees(Math.atan2(diff.getY(), diff.getX()));

    return angle;
  }

  /*
   * Returns angle to reef in degrees
   */
  public static double AngleToReef(Pose2d pose) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    Pose2d reef_center =
        alliance == Alliance.Blue
            ? PathfindingConfig.BLUE_REEF_CENTER
            : PathfindingConfig.RED_REEF_CENTER;

    return AngleToReef(pose, reef_center);
  }

  public static double ReefHeight(Pose2d pose) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double angle = AngleToReef(pose);

    if (Math.floor((30 + angle) / 60) % 2 >= 1) {
      return alliance == Alliance.Blue ? ElevatorHeights.L3 : ElevatorHeights.L2;
    }

    return alliance == Alliance.Blue ? ElevatorHeights.L2 : ElevatorHeights.L3;
  }

  public static double ReefHeight(Pose2d pose, Pose2d reefCenter) {
    Alliance alliance = Alliance.Blue;
    if (reefCenter.getMeasureX().baseUnitMagnitude() > 8) {
      alliance = Alliance.Red;
    }
    double angle = AngleToReef(pose, reefCenter);
    
    if (Math.floor((30 + Math.abs(angle)) / 60) % 2 >= 1) {
      return alliance == Alliance.Blue ? ElevatorHeights.L3 : ElevatorHeights.L2;
    }

    return alliance == Alliance.Blue ? ElevatorHeights.L2 : ElevatorHeights.L3;
  }

  public static Pose2d GetReefPose(Pose2d reefPose, Pose2d currentPose2d, double distAway) {
    double reefAngle = AngleToReef(currentPose2d, reefPose);
    double targetAngle = Math.floor((30 + reefAngle) / 60) * 60;

    double dx = Math.cos(Units.degreesToRadians(targetAngle));
    double dy = Math.sin(Units.degreesToRadians(targetAngle));

    double x = reefPose.getX() + dx * distAway;
    double y = reefPose.getY() + dy * distAway;

    return new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(180+targetAngle)));
  }

  public static Pose2d GetAllianceReefPose() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    return alliance == Alliance.Blue
            ? PathfindingConfig.BLUE_REEF_CENTER
            : PathfindingConfig.RED_REEF_CENTER;
  }

  public static Command GenerateReefCommand(Pose2d currentPos, SwerveSubsystem swerve, AutonTeleController autonTeleController, ElevatorSubsystem elevator, AlgaeEndEffectorSubsystem algaeEndEffector) {
    final Pose2d reefCenter = currentPos.getX() > 8.767 ? PathfindingConfig.BLUE_REEF_CENTER : PathfindingConfig.RED_REEF_CENTER;

    Pose2d reefStartPose = GetReefPose(reefCenter, currentPos, 2.286);
    Pose2d reefPickupPose = GetReefPose(reefCenter, currentPos, 1.294);
    Pose2d reefEndPose = GetReefPose(reefCenter, currentPos, 3.112);

    return Commands.sequence(
      Commands.either(
        Commands.sequence(
          elevator.setHeightCommand(ElevatorHeights.STOW),
          autonTeleController.GoToPose(reefStartPose, 3.0, 0.5),
          elevator.setHeightCommand(ElevatorHeights.GROUND),
          Commands.waitSeconds(0.35)
        ),
        Commands.sequence(
          elevator.setHeightCommand(ElevatorHeights.GROUND),
          new TurnToAngle(swerve, () -> AngleToReef(currentPos, reefCenter), () -> 0, () -> 0)
        ),
        () -> currentPos.getTranslation().getDistance(reefStartPose.getTranslation()) < 0.4
      ),
      elevator.setHeightCommand(ReefHeight(currentPos, reefCenter)),
      Commands.waitSeconds(0.5),
      Commands.parallel(
        algaeEndEffector.intakeUntilStalled(),
        autonTeleController.GoToPose(reefPickupPose, 1.5, 0.0)
      ),
      algaeEndEffector.holdAlgae(),
      Commands.parallel(
        Commands.sequence(
          autonTeleController.GoToPose(reefEndPose, 2.5, 2.5, 2)
        ),
        Commands.sequence(
          Commands.waitSeconds(0.5),
          elevator.setHeightCommand(ElevatorHeights.L1_25)
        )
      )
  );
  }

  public static Pose2d GetBargePose(Pose2d currentPose2d) {
    Alliance alliance = DriverStation.getAlliance().get();
    Transform2d offset = new Transform2d(new Translation2d(0, (Math.random()*2-1)/2), new Rotation2d());

    if (alliance == Alliance.Blue) {
      if (currentPose2d.getX() > 7.25) {
        return PathfindingConfig.BARGE_BLUE_FLIPPED.plus(offset);
      }
      return PathfindingConfig.BARGE_BLUE.plus(offset);
    }

    if (currentPose2d.getX() > 7.25) {
      return PathfindingConfig.BARGE_RED.plus(offset);
    }
    return PathfindingConfig.BARGE_RED_FLIPPED.plus(offset);
  }
}
