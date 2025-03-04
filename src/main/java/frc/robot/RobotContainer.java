// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControlMapping;
import frc.robot.Constants.Elevator.ElevatorHeights;
import frc.robot.subsystems.AlgaeEndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AutonTeleController;
import frc.robot.util.Logging;
import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.math.Matter;

/**
 * This class is where almost all of the robot is defined - logic and subsystems
 * are all set up
 * here.
 */
@Logged
public class RobotContainer {
  AlgaeEndEffectorSubsystem algaeSubsystem = new AlgaeEndEffectorSubsystem();
  ElevatorSubsystem m_elevator = new ElevatorSubsystem(algaeSubsystem.hasBall());
  Supplier<Matter> elevator_matter = () -> m_elevator.getMatter();
  SwerveSubsystem swerve = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"),
      () -> new Matter(new Translation3d(), 0));
  VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
  // WinchSubsystem winch = new WinchSubsystem();
  // BatteryIdentification batteryIdentification = new BatteryIdentification();

  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandGenericHID operatorController = new CommandGenericHID(2);

  DoubleSupplier swerve_x = () -> MathUtil.applyDeadband(
      -driverXbox.getRawAxis(ControlMapping.FORWARD_BACKWARD.value)
          * (1 - Math.pow((m_elevator.getHeight() / 300), 2)),
      0.02);

  DoubleSupplier swerve_y = () -> MathUtil.applyDeadband(
      -driverXbox.getRawAxis(ControlMapping.LEFT_RIGHT.value)
          * (1 - Math.pow((m_elevator.getHeight() / 300), 2)),
      0.02);

  DoubleSupplier right_stick_up_down = () -> MathUtil.applyDeadband(
      -driverXbox.getRawAxis(XboxController.Axis.kRightY.value)
          * (1 - Math.pow((m_elevator.getHeight() / 300), 2)),
      0.02);

  DoubleSupplier swerve_turn = () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(ControlMapping.TURN.value),
      0.08);

  private final SendableChooser<Command> autoChooser;

  AutonTeleController autonTeleController = new AutonTeleController(driverXbox, swerve, swerve_x, swerve_y,
      swerve_turn);

  public RobotContainer() {
    Pathfinding.setPathfinder(new LocalADStar());
    Logging.logMetadata();

    PathfindingCommand.warmupCommand().schedule();

    configureTriggers();

    Command driveCommand = swerve.driveCommand(swerve_x, swerve_y, swerve_turn);

    swerve.setDefaultCommand(driveCommand);

    NamedCommands.registerCommand(
        "Elevator_Algae_L1",
        Commands.sequence(m_elevator
            .setHeightCommand(Constants.Elevator.ElevatorHeights.GROUND)));

    NamedCommands.registerCommand(
        "Elevator_Algae_L1_25",
        Commands.sequence(
            m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1_25)));

    NamedCommands.registerCommand(
        "Elevator_Algae_L1_5",
        Commands.sequence(
            m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1_5)));

    NamedCommands.registerCommand(
        "Elevator_Algae_L2",
        Commands.sequence(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2)));

    NamedCommands.registerCommand(
        "Elevator_Algae_L3",
        Commands.sequence(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3)));

    NamedCommands.registerCommand(
        "Elevator_Algae_L4",
        Commands.sequence(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.TOP)));

    NamedCommands.registerCommand(
        "Intake_Algae",
        Commands.sequence(algaeSubsystem.intakeUntilStalled(), algaeSubsystem.holdAlgae()));

    NamedCommands.registerCommand(
        "Shoot_Algae",
        Commands.sequence(
            algaeSubsystem.startOutake(), Commands.waitSeconds(0.5),
            algaeSubsystem.stopMotors()));

    // Initialize autonomous chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Path", autoChooser);
  }

  /**
   * This method is where all of the robot's logic is defined. We link {@link
   * edu.wpi.first.wpilibj2.command.button.Trigger}s, such as controller buttons
   * and subsystem
   * state, to {@link edu.wpi.first.wpilibj2.command.Command} instances. The
   * advantage of
   * configuring all the robot's logic here is that it's easy to find, and
   * therefore easy to modify,
   * what the robot does when something happens and why.
   */
  private void configureTriggers() {
    // Controls

    // driverXbox
    // .button(ControlMapping.MOVE_TO_BARGE.value)
    // .onTrue(
    // Commands.sequence(
    // Commands.defer(
    // () ->

    // autonTeleController.GoToPose(ReefUtils.GetBargePose(swerve.getPose())),
    // Set.of(swerve)
    // ),
    // Commands.parallel(
    // new TurnToAngle(
    // swerve,
    // () -> (DriverStation.getAlliance().get() == Alliance.Blue
    // && swerve.getPose().getX() < 6.15)
    // || (DriverStation.getAlliance().get() == Alliance.Red
    // && swerve.getPose().getX() > 6.15)
    // ? 0
    // : 180,
    // swerve_x,
    // swerve_y),
    // m_elevator.setHeightCommand(ElevatorHeights.TOP)))
    // .until(() -> autonTeleController.isDriverInputting()));

    driverXbox
        .back()
        .onTrue(
            Commands.runOnce(() -> m_elevator.resetEncoder())
                .ignoringDisable(true)); // left menu button

    driverXbox.start().onTrue(swerve.zeroYawCommand().ignoringDisable(true)); // right menu button

    // driverXbox
    // .button(ControlMapping.LIFT_TO_REEF.value)
    // .onTrue(
    // Commands.parallel(
    // new RepeatCommand(
    // Commands.runOnce(
    // () -> {
    // m_elevator.setHeight(ReefUtils.ReefHeight(swerve.getPose()));
    // },
    // m_elevator)),
    // new RepeatCommand(
    // new TurnToAngle(
    // swerve,
    // () -> {
    // return ReefUtils.AngleToReef(swerve.getPose());
    // },
    // swerve_x,
    // swerve_y)))
    // .until(() -> autonTeleController.isDriverInputting()));

    driverXbox
        .axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.3)
        .whileTrue(
            new RepeatCommand(
                swerve.driveCommandRobotRelative(
                    right_stick_up_down,
                    () -> 0,
                    () -> MathUtil.applyDeadband(swerve_turn
                        .getAsDouble(), 0.1))));

    driverXbox
        .button(ControlMapping.GROUND.value)
        .onTrue(m_elevator.setHeightCommand(ElevatorHeights.GROUND));

    driverXbox.povUp().onTrue(m_elevator.setHeightCommand(ElevatorHeights.L1_5));
    driverXbox.povDown().onTrue(m_elevator.setHeightCommand(ElevatorHeights.L1_25));

    driverXbox.b().onTrue(m_elevator.setHeightCommand(ElevatorHeights.L2));

    driverXbox.x().onTrue(m_elevator.setHeightCommand(ElevatorHeights.L3));

    driverXbox.y().onTrue(m_elevator.setHeightCommand(ElevatorHeights.TOP));

    // driverXbox
    // .button(ControlMapping.CHASE_CORAL.value)
    // .whileTrue(
    // Commands.parallel(
    // m_elevator.setHeightCommand(ElevatorHeights.GROUND),
    // new AimAtAlgae(visionSubsystem, swerve)));

    driverXbox
        .button(ControlMapping.INTAKE.value)
        .onTrue(
            Commands.either(
                Commands.sequence(algaeSubsystem.intakeUntilStalled(),
                    algaeSubsystem.holdAlgae()),
                algaeSubsystem.stopMotors(),
                () -> !algaeSubsystem.hasBall().getAsBoolean()));

    driverXbox
        .button(ControlMapping.OUTTAKE.value)
        .onTrue(
            Commands.sequence(
                algaeSubsystem.startOutake(),
                Commands.waitSeconds(0.5),
                algaeSubsystem.stopMotors(),
                m_elevator.setHeightCommand(ElevatorHeights.GROUND)));

    new Trigger(HALUtil::getFPGAButton).onTrue(toggleBrakeMode().ignoringDisable(true));
  }

  public Command toggleBrakeMode() {
    return Commands.runOnce(
        () -> {
          m_elevator.toggleBrakeMode();
        },
        m_elevator);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void OnDisable() {
    m_elevator.setHeight(ElevatorHeights.GROUND);
    m_elevator.brake();
  }

  public void OnEnable() {
    m_elevator.brake();
  }
}
