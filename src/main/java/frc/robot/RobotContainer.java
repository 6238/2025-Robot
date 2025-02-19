// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControlMapping;
import frc.robot.Constants.Elevator.ElevatorHeights;
import frc.robot.subsystems.AlgaeEndEffectorSubsystem;
import frc.robot.subsystems.BatteryIdentification;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.util.Logging;
import java.io.File;

/**
 * This class is where almost all of the robot is defined - logic and subsystems are all set up
 * here.
 */
public class RobotContainer {

  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
  WinchSubsystem winch = new WinchSubsystem();
  AlgaeEndEffectorSubsystem algaeSubsystem = new AlgaeEndEffectorSubsystem();
  ElevatorSubsystem m_elevator = new ElevatorSubsystem(algaeSubsystem.hasBall());
  BatteryIdentification batteryIdentification = new BatteryIdentification();

  CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DataLogManager.start();
    Logging.logMetadata();

    batteryIdentification.startRead();

    configureTriggers();

    Command driveCommand =
        swerve.driveCommand(
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getRawAxis(ControlMapping.FORWARD_BACKWARD.value)
                        * (1 - m_elevator.getHeight() / 140),
                    0.02),
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getRawAxis(ControlMapping.LEFT_RIGHT.value)
                        * (1 - m_elevator.getHeight() / 140),
                    0.02),
            () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(ControlMapping.TURN.value), 0.08));

    swerve.setDefaultCommand(driveCommand);

    // Initialize autonomous chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Path", autoChooser);
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
    driverXbox.start().onTrue(swerve.zeroYawCommand().ignoringDisable(true));

    driverXbox
        .button(ControlMapping.ELEVATOR_L2.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2));
    driverXbox
        .button(ControlMapping.ELEVATOR_L3.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3));
    driverXbox
        .axisLessThan(
            ControlMapping.ELEVATOR_BOTTOM_TOP.value, ControlMapping.ELEVATOR_GROUND_THRESHOLD)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.GROUND));
    driverXbox
        .axisGreaterThan(
            ControlMapping.ELEVATOR_BOTTOM_TOP.value, ControlMapping.ELEVATOR_TOP_THRESHOLD)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.TOP));

    driverXbox
        .axisGreaterThan(
            ControlMapping.ELEVATOR_LOWER.value, ControlMapping.ELEVAOTR_ADJUST_THRESHOLD)
        .whileTrue(
            m_elevator.increaseHeight(
                () ->
                    -driverXbox.getRawAxis(ControlMapping.ELEVATOR_LOWER.value)
                        / ControlMapping.ELEVAOTR_ADJUST_SPEED_DECREASE));
    driverXbox
        .axisGreaterThan(
            ControlMapping.ELEVATOR_RAISE.value, ControlMapping.ELEVAOTR_ADJUST_THRESHOLD)
        .whileTrue(
            m_elevator.increaseHeight(
                () ->
                    driverXbox.getRawAxis(ControlMapping.ELEVATOR_RAISE.value)
                        / ControlMapping.ELEVAOTR_ADJUST_SPEED_DECREASE));

    driverXbox
        .button(ControlMapping.INTAKE.value)
        .onTrue(
            Commands.either(
                Commands.sequence(algaeSubsystem.intakeUntilStalled(), algaeSubsystem.holdAlgae()),
                algaeSubsystem.stopMotors(),
                algaeSubsystem.hasBall()));

    driverXbox
        .button(ControlMapping.OUTTAKE.value)
        .onTrue(
            Commands.sequence(
                algaeSubsystem.startOutake(), new WaitCommand(0.5), algaeSubsystem.stopMotors()));

    driverXbox.povUp().onTrue(winch.toGrab());
    driverXbox.povDown().onTrue(winch.toPull());

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
