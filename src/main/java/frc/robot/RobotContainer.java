// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Elevator.ElevatorHeights;
import frc.robot.subsystems.AlgaeEndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.util.AutonTeleController;
import frc.robot.util.Logging;
import frc.robot.util.OrcestraManager;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.math.Matter;

/**
 * This class is where almost all of the robot is defined - logic and subsystems are all set up
 * here.
 */
@Logged
public class RobotContainer {
  AlgaeEndEffectorSubsystem algaeSubsystem = new AlgaeEndEffectorSubsystem();
  public ElevatorSubsystem m_elevator = new ElevatorSubsystem(algaeSubsystem.hasBall());
  Supplier<Matter> elevator_matter = () -> m_elevator.getMatter();
  SwerveSubsystem swerve =
      new SwerveSubsystem(
          new File(Filesystem.getDeployDirectory(), "swerve2"),
          () -> new Matter(new Translation3d(), 0));
  VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
  WinchSubsystem winch = new WinchSubsystem();
  LEDSubsystem led = new LEDSubsystem();
  // BatteryIdentification batteryIdentification = new BatteryIdentification();

  private static boolean manualModeEnabled = false;

  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandGenericHID operatorController = new CommandGenericHID(2);

  DoubleSupplier swerve_x =
      () ->
          MathUtil.applyDeadband(
              -driverXbox.getLeftY()
                  * (1 - Math.pow((m_elevator.getHeight() / 300), 2))
                  * (driverXbox.getRightTriggerAxis() > 0.5 ? 0.5 : 1.0),
              0.02);

  DoubleSupplier swerve_y =
      () ->
          MathUtil.applyDeadband(
              -driverXbox.getLeftX()
                  * (1 - Math.pow((m_elevator.getHeight() / 300), 2))
                  * (driverXbox.getRightTriggerAxis() > 0.5 ? 0.5 : 1.0),
              0.02);

  DoubleSupplier right_stick_up_down =
      () ->
          MathUtil.applyDeadband(
              -driverXbox.getRawAxis(XboxController.Axis.kRightY.value)
                  * (1 - Math.pow((m_elevator.getHeight() / 300), 2))
                  * (driverXbox.getRightTriggerAxis() > 0.5 ? 0.5 : 1.0),
              0.02);

  DoubleSupplier swerve_turn =
      () ->
          MathUtil.applyDeadband(
              -driverXbox.getRightX() * (driverXbox.getRightTriggerAxis() > 0.5 ? 0.5 : 1.0), 0.08);

  private final SendableChooser<Command> autoChooser;

  AutonTeleController autonTeleController =
      new AutonTeleController(driverXbox, swerve, swerve_x, swerve_y, swerve_turn);

  public RobotContainer() {
    Logging.logMetadata();

    configureTriggers();

    Command driveCommand = swerve.driveCommand(swerve_x, swerve_y, swerve_turn);

    swerve.setDefaultCommand(driveCommand);

    NamedCommands.registerCommand(
        "Elevator_Algae_L1",
        m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.GROUND));

    NamedCommands.registerCommand(
        "Elevator_Algae_L1_25",
        m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1_25));

    NamedCommands.registerCommand(
        "Elevator_Algae_L1_5",
        m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1_5));

    NamedCommands.registerCommand(
        "Elevator_Algae_L2", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2));

    NamedCommands.registerCommand(
        "Elevator_Algae_L3", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3));

    NamedCommands.registerCommand(
        "Elevator_Algae_L4", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.TOP));

    NamedCommands.registerCommand("Elevator_Choral_L1", m_elevator.setHeightCommand(22));

    NamedCommands.registerCommand(
        "Intake_Algae",
        Commands.sequence(
            algaeSubsystem.intakeUntilStalled().withTimeout(3), algaeSubsystem.holdAlgae()));

    NamedCommands.registerCommand("Start_Intake", algaeSubsystem.startIntake());

    NamedCommands.registerCommand("Stop_Intake", algaeSubsystem.holdAlgae());

    NamedCommands.registerCommand(
        "Shoot_Algae",
        Commands.sequence(
            algaeSubsystem.startOutake(), Commands.waitSeconds(0.5), algaeSubsystem.stopMotors()));

    NamedCommands.registerCommand(
        "Shoot_Choral",
        Commands.sequence(
            algaeSubsystem.startFastOutake(),
            Commands.waitSeconds(0.5),
            algaeSubsystem.stopMotors()));

    new EventTrigger("Elevator_Algae_L2")
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2));
    new EventTrigger("Elevator_Algae_L3")
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3));
    new EventTrigger("Elevator_Algae_L4")
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.TOP));
    new EventTrigger("Elevator_Algae_L1")
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.GROUND));
    new EventTrigger("Elevator_Choral_L1").onTrue(m_elevator.setHeightCommand(25));
    new EventTrigger("Intake")
        .onTrue(Commands.sequence(algaeSubsystem.intakeUntilStalled(), algaeSubsystem.holdAlgae()));
    new EventTrigger("Start_Intake").onTrue(algaeSubsystem.startIntake());
    new EventTrigger("Stop_Intake").onTrue(algaeSubsystem.holdAlgae());
    new EventTrigger("Shoot").onTrue(algaeSubsystem.startOutake());

    Pathfinding.setPathfinder(new LocalADStar());
    PathfindingCommand.warmupCommand().schedule();

    OrcestraManager.getInstance().load("acdc.chrp");

    // Initialize autonomous chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Path", autoChooser);

    SmartDashboard.putBoolean("ManualModeEnabled", manualModeEnabled);
  }

  /**
   * This method is where all of the robot's logic is defined. We link {@link
   * edu.wpi.first.wpilibj2.command.button.Trigger}s, such as controller buttons and subsystem
   * state, to {@link edu.wpi.first.wpilibj2.command.Command} instances. The advantage of
   * configuring all the robot's logic here is that it's easy to find, and therefore easy to modify,
   * what the robot does when something happens and why.
   */
  private void configureTriggers() {
    // Controls

    new Trigger(algaeSubsystem.hasBall())
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> driverXbox.setRumble(RumbleType.kBothRumble, 1)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> driverXbox.setRumble(RumbleType.kBothRumble, 0))));

    driverXbox
        .back()
        .onTrue(
            Commands.runOnce(() -> m_elevator.resetEncoder())
                .ignoringDisable(true)); // left menu button
    driverXbox.start().onTrue(swerve.zeroYawCommand().ignoringDisable(true)); // right menu button

    driverXbox
        .axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.3)
        .whileTrue(
            new RepeatCommand(
                swerve.driveCommandRobotRelative(
                    right_stick_up_down,
                    () -> 0,
                    () -> MathUtil.applyDeadband(swerve_turn.getAsDouble(), 0.1))));

    SmartDashboard.putBoolean("RAISE_CLIMBER", false);
    new Trigger(() -> SmartDashboard.getBoolean("RAISE_CLIMBER", false))
        .whileTrue(Commands.run(() -> winch.lower(), winch));

    new Trigger(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() < 30)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> led.setAnimation(LEDMode.OFF)),
                Commands.run(() -> winch.lower(), winch)));

    // driverXbox
    //     .leftTrigger()
    //     .onTrue(
    //         Commands.defer(
    //             () -> autonTeleController.GoToPose(ReefUtils.GetBargePose(swerve.getPose())),
    //             Set.of(swerve)));
    
    driverXbox.leftStick().onTrue(
      Commands.sequence(
        m_elevator.setHeightCommand(ElevatorHeights.STOW),
        autonTeleController.GoToPose(new Pose2d(1.874, 3.971, new Rotation2d(Math.toRadians(90))), 2.0, 1.0),
        Commands.parallel(
          algaeSubsystem.startIntake(),
          m_elevator.setHeightCommand(ElevatorHeights.L2),
          autonTeleController.GoToPose(new Pose2d(2.874, 3.971, new Rotation2d(Math.toRadians(90))), 1, 0.0)
        ),
        Commands.waitSeconds(0.25),
        algaeSubsystem.stopMotors(),
        Commands.parallel(
          Commands.sequence(
            autonTeleController.GoToPose(new Pose2d(1.374, 3.971, new Rotation2d(Math.toRadians(90))), 2.5, 2.5, 1.5)
          ),
          Commands.sequence(
            Commands.waitSeconds(0.1),
            m_elevator.setHeightCommand(ElevatorHeights.L1_25)
          )
        )
    ));

    // driverXbox
    // .x()
    // .onTrue(
    // Commands.either(
    // m_elevator.setHeightCommand(ElevatorHeights.L3)
    // Commands.sequence(
    // Commands.parallel(
    // Commands.defer(
    // () -> autonTeleController.GoToPose(ReefUtils.GetBargePose(swerve.getPose())),
    // Set.of(swerve))),
    // new TurnToAngle(
    // swerve,
    // () -> (DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : 90,
    // swerve_x,
    // swerve_y),
    // m_elevator.setHeightCommand(ElevatorHeights.L1_25),
    // Commands.waitSeconds(0.15),
    // Commands.sequence(
    // algaeSubsystem.startOutake(),
    // Commands.waitSeconds(0.5),
    // algaeSubsystem.stopMotors(),
    // m_elevator.setHeightCommand(ElevatorHeights.GROUND)))
    // .until(() -> autonTeleController.isDriverInputting()),
    // () -> manualModeEnabled));
    // );

    // driverXbox.rightStick().onTrue(m_elevator.setHeightCommand(25));

    driverXbox
        .y()
        .onTrue(
            Commands.sequence(
                m_elevator.setHeightCommand(ElevatorHeights.GROUND),
                Commands.waitUntil(() -> m_elevator.getHeight() > 4),
                m_elevator.setHeightCommand(ElevatorHeights.TOP)));

    driverXbox
        .x()
        .onTrue(
            Commands.sequence(
                m_elevator.setHeightCommand(ElevatorHeights.GROUND),
                Commands.waitUntil(() -> m_elevator.getHeight() > 4),
                m_elevator.setHeightCommand(ElevatorHeights.L3)));

    driverXbox
        .b()
        .onTrue(
            Commands.sequence(
                m_elevator.setHeightCommand(ElevatorHeights.GROUND),
                Commands.waitUntil(() -> m_elevator.getHeight() > 4),
                m_elevator.setHeightCommand(ElevatorHeights.L2)));

    driverXbox
        .povLeft()
        .onTrue(
            Commands.sequence(
                m_elevator.setHeightCommand(ElevatorHeights.GROUND),
                Commands.waitUntil(() -> m_elevator.getHeight() > 4),
                m_elevator.setHeightCommand(ElevatorHeights.L1_25)));

    driverXbox
        .povRight()
        .onTrue(
            Commands.sequence(
                m_elevator.setHeightCommand(ElevatorHeights.GROUND),
                Commands.waitUntil(() -> m_elevator.getHeight() > 4),
                m_elevator.setHeightCommand(ElevatorHeights.L1_5)));

    driverXbox.a().whileTrue(m_elevator.setHeightCommand(ElevatorHeights.GROUND));

    // Commands.either(
    // Commands.parallel(
    // algaeSubsystem.intakeUntilStalled(),
    // Commands.runOnce(() -> {
    // m_elevator.setHeight(ReefUtils.ReefHeight(swerve.getPose()));
    // })
    // ),
    // new RepeatCommand(
    // Commands.runOnce(
    // () -> {
    // },
    // m_elevator))),
    // new AimAtAlgae(visionSubsystem, swerve)
    // new TurnToAngle(
    // swerve,
    // () -> {
    // return ReefUtils.AngleToReef(swerve.getPose());
    // },
    // swerve_x,
    // swerve_y
    // )
    // ).until(() -> autonTeleController.isDriverInputting()),

    // new RepeatCommand(
    // new TurnToAngle(
    // swerve,
    // () -> {
    // return ReefUtils.AngleToReef(swerve.getPose());
    // },
    // swerve_x,
    // swerve_y)))
    // ,
    // () -> manualModeEnabled));

    // driverXbox.leftTrigger().onTrue(Commands.parallel(
    // m_elevator.setHeightCommand(ElevatorHeights.GROUND),
    // new AimAtAlgae(visionSubsystem, swerve)));

    driverXbox
        .povDown()
        .whileTrue(Commands.run(() -> winch.lower(), winch)); // must be run repeatedly
    driverXbox.povDown().onFalse(Commands.runOnce(() -> winch.stopMotor(), winch));
    driverXbox
        .povDown()
        .onTrue(led.setAnimationToAllianceColorCommand(DriverStation.getAlliance()));

    driverXbox
        .povUp()
        .whileTrue(Commands.run(() -> winch.raise(), winch)); // must be run repeatedly
    driverXbox.povUp().onFalse(Commands.runOnce(() -> winch.stopMotor(), winch));
    driverXbox.povUp().onTrue(led.climbCommand());

    driverXbox.leftTrigger().onTrue(m_elevator.setHeightCommand(ElevatorHeights.STOW));
    driverXbox.rightTrigger().onTrue(m_elevator.setHeightCommand(ElevatorHeights.L1_25));

    // driverXbox
    //     .leftBumper()
    //     .onTrue(
    //         algaeSubsystem.startIntake());

    // driverXbox.leftBumper().onFalse(
    // 	algaeSubsystem.reverse()
    // );

    driverXbox
        .leftBumper()
        .onTrue(
            Commands.sequence(
                algaeSubsystem.intakeUntilStalled(),
                Commands.waitSeconds(0.1),
                algaeSubsystem.holdAlgae()));

    driverXbox
        .rightBumper()
        .onTrue(
            Commands.sequence(
                algaeSubsystem.startOutake(),
                Commands.waitSeconds(0.5),
                algaeSubsystem.stopMotors(),
                Commands.deferredProxy(
                    () -> {
                      if (m_elevator.getHeight() >= ElevatorHeights.TOP - 1) {
                        return m_elevator.setHeightCommand(ElevatorHeights.GROUND);
                      }
                      return Commands.none();
                    })));

    new Trigger(HALUtil::getFPGAButton).onTrue(toggleBrakeMode().ignoringDisable(true));

    // spotless:off
    new Trigger(algaeSubsystem.hasBall())
        .onTrue(led.indicateIntookCommand())
        .onFalse(led.setAnimationToAllianceColorCommand(DriverStation.getAlliance()));
    // spotless:on
  }

  public Command toggleBrakeMode() {
    return Commands.runOnce(
        () -> {
          m_elevator.toggleBrakeMode();
          OrcestraManager.getInstance().getOrchestra().play();
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
