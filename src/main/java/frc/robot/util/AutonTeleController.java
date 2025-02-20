package frc.robot.util;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControlMapping;
import frc.robot.Constants.PathfindingConfig;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonTeleController {
    CommandXboxController xboxEmulator = new CommandXboxController(1);
    SwerveSubsystem swerveSubsystem;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    DoubleSupplier turnSupplier;


    public AutonTeleController(SwerveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier turnSupplier) {
        swerveSubsystem = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.turnSupplier = turnSupplier;
    }

    public void SetupPoseCommands() {
        xboxEmulator.button(ControlMapping.MOVE_TO_BARGE_BUTTON.value).onTrue(Commands.either(
            GoToPose(PathfindingConfig.BARGE_TOP),
            GoToPose(PathfindingConfig.BARGE_BOTTOM),
            () -> swerveSubsystem.getPose().getY() >= 4
        ));
    }

    public boolean isDriverInputting() {
        return Math.abs(xSupplier.getAsDouble()) <= PathfindingConfig.DRIVE_RESUME_DEADBAND &&
            Math.abs(xSupplier.getAsDouble()) <= PathfindingConfig.DRIVE_RESUME_DEADBAND &&
            Math.abs(xSupplier.getAsDouble()) <= PathfindingConfig.DRIVE_RESUME_DEADBAND;
    }

    public Command GoToPose(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                2.0, 3.0,
                Units.degreesToRadians(360), Units.degreesToRadians(540));

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0
        );
        return pathfindingCommand.until(() -> isDriverInputting());
    }
}
