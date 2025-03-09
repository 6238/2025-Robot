// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DataLogManager;
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
import frc.robot.Constants.Elevator.ElevatorHeights;
import frc.robot.commands.AimAtAlgae;
import frc.robot.subsystems.AlgaeEndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.util.AutonTeleController;
import frc.robot.util.Logging;
import frc.robot.util.ReefUtils;
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
			new File(Filesystem.getDeployDirectory(), "swerve2"),
			() -> new Matter(new Translation3d(), 0));
	VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
	WinchSubsystem winch = new WinchSubsystem();
	// BatteryIdentification batteryIdentification = new BatteryIdentification();

	private static boolean manualModeEnabled = false;

	CommandXboxController driverXbox = new CommandXboxController(0);
	CommandGenericHID operatorController = new CommandGenericHID(2);

	DoubleSupplier swerve_x = () -> MathUtil.applyDeadband(
			-driverXbox.getLeftY()
					* (1 - Math.pow((m_elevator.getHeight() / 300), 2)),
			0.02);

	DoubleSupplier swerve_y = () -> MathUtil.applyDeadband(
			-driverXbox.getLeftX()
					* (1 - Math.pow((m_elevator.getHeight() / 300), 2)),
			0.02);

	DoubleSupplier right_stick_up_down = () -> MathUtil.applyDeadband(
			-driverXbox.getRawAxis(XboxController.Axis.kRightY.value)
					* (1 - Math.pow((m_elevator.getHeight() / 300), 2)),
			0.02);

	DoubleSupplier swerve_turn = () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.08);

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
				"Elevator_Algae_L1", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.GROUND));

		NamedCommands.registerCommand(
				"Elevator_Algae_L1_25", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1_25));

		NamedCommands.registerCommand(
				"Elevator_Algae_L1_5", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1_5));

		NamedCommands.registerCommand(
				"Elevator_Algae_L2", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2));

		NamedCommands.registerCommand(
				"Elevator_Algae_L3", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3));

		NamedCommands.registerCommand(
				"Elevator_Algae_L4", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.TOP));

		NamedCommands.registerCommand(
				"Elevator_Choral_L1", m_elevator.setHeightCommand(25));

		NamedCommands.registerCommand(
				"Intake_Algae",
				Commands.sequence(Commands.runOnce(() -> DataLogManager.log("Intake Started")), algaeSubsystem.intakeUntilStalled(), algaeSubsystem.holdAlgae(), Commands.runOnce(() -> DataLogManager.log("Intake Started"))));

		NamedCommands.registerCommand(
				"Shoot_Algae",
				Commands.sequence(
						algaeSubsystem.startOutake(), Commands.waitSeconds(0.5), algaeSubsystem.stopMotors()));

		// Initialize autonomous chooser
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auton Path", autoChooser);

		SmartDashboard.putBoolean("ManualModeEnabled", manualModeEnabled);
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

		driverXbox.rightStick().onTrue(Commands.run(() -> manualModeEnabled = !manualModeEnabled));
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

		driverXbox.leftTrigger().onTrue(Commands.defer(
			() -> autonTeleController.GoToPose(ReefUtils.GetBargePose(swerve.getPose())),
			Set.of(swerve)));

		driverXbox
				.y()
				.onTrue(
					m_elevator.setHeightCommand(ElevatorHeights.TOP));

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

		driverXbox
				.x()
				.onTrue(
						m_elevator.setHeightCommand(ElevatorHeights.L3));

		driverXbox
				.b()
				.onTrue(
						m_elevator.setHeightCommand(ElevatorHeights.L2));
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

		driverXbox
				.a()
				.whileTrue(
						Commands.either(
								m_elevator.setHeightCommand(ElevatorHeights.GROUND),
								Commands.parallel(
										m_elevator.setHeightCommand(ElevatorHeights.GROUND),
										new AimAtAlgae(visionSubsystem, swerve)),
								() -> manualModeEnabled));

		// driverXbox.leftTrigger().onTrue(Commands.parallel(
		// m_elevator.setHeightCommand(ElevatorHeights.GROUND),
		// new AimAtAlgae(visionSubsystem, swerve)));

		driverXbox.povLeft().onTrue(m_elevator.setHeightCommand(ElevatorHeights.L1_25));
		driverXbox.povRight().onTrue(m_elevator.setHeightCommand(ElevatorHeights.L1_5));

		driverXbox.povDown().onTrue(winch.toGrab());
		driverXbox.povUp().onTrue(winch.toPull());

		driverXbox
				.leftBumper()
				.onTrue(
						Commands.either(
								Commands.sequence(algaeSubsystem.intakeUntilStalled(), algaeSubsystem.holdAlgae()),
								algaeSubsystem.stopMotors(),
								() -> !algaeSubsystem.hasBall().getAsBoolean()));

		driverXbox
				.rightBumper()
				.onTrue(
						Commands.sequence(
								algaeSubsystem.startOutake(),
								Commands.waitSeconds(0.5),
								algaeSubsystem.stopMotors()));

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
