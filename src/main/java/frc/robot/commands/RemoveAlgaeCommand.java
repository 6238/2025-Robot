package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeEndEffector;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

@Logged
public class RemoveAlgaeCommand extends Command {
  private final SwerveSubsystem m_driveSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;

  private Pose2d startPose;
  private double startHeight;
  private double dist;
  private DoubleSupplier speed_supplier;

  public RemoveAlgaeCommand(
      SwerveSubsystem driveSubsystem, ElevatorSubsystem elevator, DoubleSupplier speed) {
    m_driveSubsystem = driveSubsystem;
    m_elevatorSubsystem = elevator;
    addRequirements(m_driveSubsystem, m_elevatorSubsystem);
    speed_supplier = speed;

    startPose = new Pose2d();
  }

  @Override
  public void initialize() {
    startPose = m_driveSubsystem.getPose();
    startHeight =
        m_elevatorSubsystem.getHeight() / Constants.Elevator.ElevatorHeights.ELEVATOR_GEAR_RATIO;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(
        new Translation2d(-AlgaeEndEffector.REEF_REMOVAL_SPEED * speed_supplier.getAsDouble(), 0),
        0,
        false);

    Transform2d diff = startPose.minus(m_driveSubsystem.getPose());
    dist = Math.sqrt(Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2));
    m_elevatorSubsystem.setHeight(
        Units.metersToInches(dist * Math.sin(Math.toRadians(35))) / 6 + startHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dist > AlgaeEndEffector.REEF_REMOVAL_DIST.baseUnitMagnitude()
        || speed_supplier.getAsDouble() < 0.1;
  }
}
