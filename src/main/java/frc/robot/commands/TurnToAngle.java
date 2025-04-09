package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

@Logged
public class TurnToAngle extends Command {
  DoubleSupplier targetAngle;
  SwerveSubsystem swerve;
  DoubleSupplier swerve_x;
  DoubleSupplier swerve_y;
  Boolean fieldRelative = true;

  PIDController turn_pid = new PIDController(0.35, 0.0005,  0.04);

  public TurnToAngle(
      SwerveSubsystem swerve,
      DoubleSupplier targetAngle,
      DoubleSupplier swerve_x,
      DoubleSupplier swerve_y,
      Boolean diasbleFieldRelative) {
    this.swerve_x = swerve_x;
    this.swerve_y = swerve_y;
    this.targetAngle = targetAngle;
    this.swerve = swerve;
    this.fieldRelative = !diasbleFieldRelative;

    turn_pid.setTolerance(5);
  }

  public TurnToAngle(
      SwerveSubsystem swerve,
      DoubleSupplier targetAngle,
      DoubleSupplier swerve_x,
      DoubleSupplier swerve_y) {
    this(swerve, targetAngle, swerve_x, swerve_y, false);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double turnValue =
        turn_pid.calculate(swerve.getPose().getRotation().getDegrees(), targetAngle.getAsDouble());

    swerve.drive(
        new Translation2d(swerve_x.getAsDouble(), swerve_y.getAsDouble()),
        turnValue,
        fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(
        new Translation2d(swerve_x.getAsDouble(), swerve_y.getAsDouble()), 0, fieldRelative);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(targetAngle.getAsDouble() - swerve.getPose().getRotation().getDegrees())
        <= Swerve.TURN_THRESHOLD;
  }
}
