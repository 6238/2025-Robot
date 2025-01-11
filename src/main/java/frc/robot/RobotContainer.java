// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OuttakeGains;
import frc.robot.Constants.Arm.ArmStates;
import frc.robot.Subsystems.SwerveSubsystem;
import java.io.File;

import org.ejml.dense.row.decompose.TriangularSolver_CDRM;
import org.ejml.equation.ManagerFunctions.Input1;

/**
 * This class is where almost all of the robot is defined - logic and subsystems are all set up
 * here.
 */
public class RobotContainer {

  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  CommandXboxController driverXbox = new CommandXboxController(0);

  // private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    configureTriggers();

    swerve.setDefaultCommand(
        swerve.driveCommand(
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getLeftY(),
                    0.02), // Y axis on joystick is X axis for FRC. Forward is postive-Y, so need to
            // invert sign
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getLeftX(),
                    0.02), // X axis on joystick is Y axis for FRC. Left is positive-X, so need to
            // invert sign
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getRightX(), // Rotation for FRC is CCW-positive, so need to invert sign
                    0.08)));
    // Initialize autonomous chooser
    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auton Path", autoChooser);

    // Log metadata
    //MetadataLogger.logMetadata();
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
    driverXbox.start().onTrue(swerve.zeroYawCommand());
    

    // Intaking sequence - run each of these commands sequentially.
    // TODO: This sequence takes up the whole of subsystems. Triggers would be better.
 

  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}