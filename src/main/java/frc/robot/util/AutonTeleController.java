package frc.robot.util;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PathfindingConfig;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonTeleController {
  CommandXboxController driverXbox;
  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  DoubleSupplier turnSupplier;

  public AutonTeleController(
      CommandXboxController driverXbox,
      SwerveSubsystem swerve,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier turnSupplier) {
    this.driverXbox = driverXbox;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.turnSupplier = turnSupplier;
  }

  public boolean isDriverInputting() {
    return Math.abs(xSupplier.getAsDouble()) > PathfindingConfig.DRIVE_RESUME_DEADBAND
        && Math.abs(xSupplier.getAsDouble()) > PathfindingConfig.DRIVE_RESUME_DEADBAND
        && Math.abs(xSupplier.getAsDouble()) > PathfindingConfig.DRIVE_RESUME_DEADBAND;
  }

  public Command GoToPose(Pose2d targetPose) {
    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(540));

    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);

    return pathfindingCommand;
  }

  public Command GoToPose(Pose2d targetPose, double maxSpeed) {
    PathConstraints constraints =
        new PathConstraints(maxSpeed, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(540));

    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);

    return pathfindingCommand;
  }

  public Command GoToPose(Pose2d targetPose, double maxSpeed, double targetEndVelocity) {
    PathConstraints constraints =
        new PathConstraints(maxSpeed, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(540));

    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, targetEndVelocity);

    return pathfindingCommand;
  }

  public Command GoToPose(Pose2d targetPose, double maxSpeed, double targetEndVelocity, double maxAcceleration) {
    PathConstraints constraints =
        new PathConstraints(maxSpeed, maxAcceleration, Units.degreesToRadians(360), Units.degreesToRadians(540));

    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, targetEndVelocity);

    return pathfindingCommand;
  }
}
