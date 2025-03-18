// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Elevator.ElevatorHeights;
import static frc.robot.Constants.Swerve.MaxAngularRate;
import static frc.robot.Constants.Swerve.MaxSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeEndEffectorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.telemetry.Telemetry;
import frc.robot.util.Logging;
import frc.robot.util.OrcestraManager;

/**
 * This class is where almost all of the robot is defined - logic and subsystems are all set up
 * here.
 */
@Logged
public class RobotContainer {
  AlgaeEndEffectorSubsystem algaeSubsystem = new AlgaeEndEffectorSubsystem();
  public ElevatorSubsystem m_elevator = new ElevatorSubsystem(algaeSubsystem.hasBall());
  WinchSubsystem winch = new WinchSubsystem();
  LEDSubsystem led = new LEDSubsystem();
  // BatteryIdentification batteryIdentification = new BatteryIdentification();

  @NotLogged CommandXboxController driverXbox = new CommandXboxController(0);

  /* Setting up bindings for necessary control of the swerve drive platform */
  @NotLogged
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  @NotLogged private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  @NotLogged private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  @NotLogged private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  @NotLogged private final Telemetry logger = new Telemetry(MaxSpeed);

  @NotLogged public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    Logging.logMetadata();

    setupPathPlanner();
    // configureTriggers();

    drivetrain.registerTelemetry(logger::telemeterize);
    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverXbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    driverXbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverXbox.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))
    ));

    driverXbox.pov(0).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0.5).withVelocityY(0))
    );
    driverXbox.pov(180).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-0.5).withVelocityY(0))
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverXbox.back().and(driverXbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverXbox.back().and(driverXbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverXbox.start().and(driverXbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverXbox.start().and(driverXbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driverXbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Initialize autonomous chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Path", autoChooser);


    OrcestraManager.getInstance().load("acdc.chrp");
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
    
    SmartDashboard.putBoolean("RAISE_CLIMBER", false);
    new Trigger(() -> SmartDashboard.getBoolean("RAISE_CLIMBER", false)).whileTrue(Commands.run(() -> winch.lower(), winch));

    new Trigger(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() < 30).onTrue(Commands.sequence(
        Commands.runOnce(() -> led.setAnimation(LEDMode.OFF)),
        Commands.run(() -> winch.lower(), winch)
    ));

    driverXbox.rightStick().onTrue(m_elevator.setHeightCommand(25));

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

    driverXbox
        .povDown()
        .whileTrue(Commands.run(() -> winch.lower(), winch)); // must be run repeatedly
    driverXbox.povDown().onFalse(Commands.runOnce(() -> winch.stopMotor(), winch));
    driverXbox.povDown().onTrue(led.setAnimationToAllianceColorCommand(DriverStation.getAlliance()));

    driverXbox
        .povUp()
        .whileTrue(Commands.run(() -> winch.raise(), winch)); // must be run repeatedly
    driverXbox.povUp().onFalse(Commands.runOnce(() -> winch.stopMotor(), winch));
    driverXbox.povUp().onTrue(led.climbCommand());

    driverXbox.leftTrigger().onTrue(m_elevator.setHeightCommand(ElevatorHeights.STOW));

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

  private void setupPathPlanner() {
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
    
    NamedCommands.registerCommand(
        "Start_Intake",
        algaeSubsystem.startIntake());

    NamedCommands.registerCommand(
        "Stop_Intake",
        algaeSubsystem.holdAlgae());

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
  }
}
