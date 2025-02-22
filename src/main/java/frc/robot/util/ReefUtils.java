package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoMoveGeneration;
import frc.robot.Constants.Elevator.ElevatorHeights;

public class ReefUtils {

  public static double AngleToReef(Pose2d pose) {
    Alliance alliance = DriverStation.getAlliance().get();

    Pose2d reef_center_blue =
        alliance == Alliance.Blue
            ? AutoMoveGeneration.REEF_CENTER_BLUE
            : AutoMoveGeneration.REEF_CENTER_RED;
    Transform2d diff = pose.minus(reef_center_blue);

    return Math.atan2(diff.getY(), diff.getX());
  }

  public static double ReefHeight(Pose2d pose) {
    Alliance alliance = DriverStation.getAlliance().get();
    double angle = AngleToReef(pose);

    if ((30 + angle) / 60 % 2 >= 1) {
      return alliance == Alliance.Blue ? ElevatorHeights.L3 : ElevatorHeights.L2;
    }

    return alliance == Alliance.Blue ? ElevatorHeights.L2 : ElevatorHeights.L3;
  }
}
