package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class TurnToAngle extends Command {
  DoubleSupplier targetAngle;
  SwerveSubsystem swerve;

  PIDController turn_pid = new PIDController(Swerve.TURN_kP, Swerve.TURN_kI, Swerve.TURN_kD);

  public TurnToAngle(
      SwerveSubsystem swerve,
      DoubleSupplier targetAngle,
      DoubleSupplier swerve_x,
      DoubleSupplier swerve_y) {
    this.targetAngle = targetAngle;
    this.swerve = swerve;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double turnValue =
        turn_pid.calculate(swerve.getPose().getRotation().getDegrees(), targetAngle.getAsDouble());

    swerve.drive(new Translation2d(), turnValue, true);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(targetAngle.getAsDouble() - swerve.getPose().getRotation().getDegrees())
        <= Swerve.TURN_THRESHOLD;
  }
}
