// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.Swerve.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAX_SPEED;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DrivingRate;
import frc.robot.util.DrivingRate.DrivingRateConfig;
import frc.robot.util.Logging;
import swervelib.math.Matter;

/**
 * This class is where almost all of the robot is defined - logic and subsystems
 * are all set up
 * here.
 */
@Logged
public class RobotContainer {
  SwerveSubsystem swerve = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"),
      () -> new Matter(new Translation3d(), 0));

  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController manualController = new CommandXboxController(1);

  DrivingRateConfig TRANSLATE_RATE_CONFIG = new DrivingRateConfig(MAX_SPEED / 1.75, MAX_SPEED, 0.3);
  DrivingRateConfig TURN_RATE_CONFIG = new DrivingRateConfig(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY * 2, 0.3);

  DoubleSupplier swerve_x = () -> DrivingRate.applyRateConfig(-MathUtil.applyDeadband(driverXbox.getLeftY(), 0.02),
      TRANSLATE_RATE_CONFIG) + DrivingRate.applyRateConfig(-MathUtil.applyDeadband(manualController.getLeftY(), 0.02),
      TRANSLATE_RATE_CONFIG);

  DoubleSupplier swerve_y = () -> DrivingRate.applyRateConfig(-MathUtil.applyDeadband(driverXbox.getLeftX(), 0.02),
      TRANSLATE_RATE_CONFIG) + DrivingRate.applyRateConfig(-MathUtil.applyDeadband(manualController.getLeftX(), 0.02),
      TRANSLATE_RATE_CONFIG);

  DoubleSupplier swerve_turn = () -> DrivingRate
      .applyRateConfig(-MathUtil.applyDeadband(driverXbox.getRightX(), 0.02), TURN_RATE_CONFIG) + DrivingRate
      .applyRateConfig(-MathUtil.applyDeadband(manualController.getRightX(), 0.02), TURN_RATE_CONFIG);

  public RobotContainer() {
    Logging.logMetadata();

    configureTriggers();

    Command driveCommand = swerve.driveCommand(swerve_x, swerve_y, swerve_turn);

    swerve.setDefaultCommand(driveCommand);
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

    driverXbox.start().onTrue(swerve.zeroYawCommand().ignoringDisable(true)); // right menu button
    manualController.start().onTrue(swerve.zeroYawCommand().ignoringDisable(true)); // right menu button

    new Trigger(HALUtil::getFPGAButton).onTrue(toggleBrakeMode().ignoringDisable(true));
  }

  public Command toggleBrakeMode() {
    return Commands.none();
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public void OnDisable() {
  }

  public void OnEnable() {
  }
}
