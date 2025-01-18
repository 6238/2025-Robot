package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PathfindingConfig;

public class AutonTeleController {
    CommandXboxController xboxEmulator = new CommandXboxController(1);
    CommandXboxController driverController;

    public AutonTeleController(CommandXboxController driverController) {
        this.driverController = driverController;
        SetupPoseCommands();
    }

    public void SetupPoseCommands() {
        xboxEmulator.a().onTrue(GoToPose(PathfindingConfig.SOURCE_ONE));
        xboxEmulator.b().onTrue(GoToPose(PathfindingConfig.SOURCE_TWO));
    }

    public boolean isDriverInputting() {
        return 0 == MathUtil.applyDeadband(-driverController.getLeftY(), PathfindingConfig.DRIVE_RESUME_DEADBAND) +
               MathUtil.applyDeadband(-driverController.getLeftX(), PathfindingConfig.DRIVE_RESUME_DEADBAND) +
               MathUtil.applyDeadband(-driverController.getRightX(), PathfindingConfig.TURN_RESUME_DEADBAND) +
               MathUtil.applyDeadband(-driverController.getRightY(), PathfindingConfig.TURN_RESUME_DEADBAND);
    }

    public Command GoToPose(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0
        );
        return pathfindingCommand.until(() -> isDriverInputting());
    }
}
