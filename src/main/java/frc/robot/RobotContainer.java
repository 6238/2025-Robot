// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RemoveAlgaeCommand;
import frc.robot.subsystems.AlgaeEndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Logging;

/**
 * This class is where almost all of the robot is defined - logic and subsystems are all set up
 * here.
 */
public class RobotContainer {

  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
  ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  AlgaeEndEffectorSubsystem algaeSubsystem = new AlgaeEndEffectorSubsystem();

  CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DataLogManager.start();
    Logging.logMetadata();
    Logging.initializeCommandSchedulerHooks();

    configureTriggers();
    
    NamedCommands.registerCommand ("Elevator_Algae_L2", Commands.sequence(
      m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3)
    ));

    NamedCommands.registerCommand ("Elevator_Algae_L2", Commands.sequence(
      m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L4)
    ));

    NamedCommands.registerCommand("Reverse_From_Reef", new RemoveAlgaeCommand(swerve, m_elevator, () -> Constants.AlgaeEndEffector.REEF_REMOVAL_CONTROLLER_VAL));

    NamedCommands.registerCommand("Intake_Algae", Commands.sequence(
      algaeSubsystem.intakeUntilStalled(),
      algaeSubsystem.holdAlgae()
    ));

    NamedCommands.registerCommand("Shoot_Algae", Commands.sequence(
      algaeSubsystem.startOutake(),
      new WaitCommand(0.5),
      algaeSubsystem.stopMotors()
    ));

    Command driveCommand = swerve.driveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.02),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.02),
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.08));
      
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
    driverXbox.start().onTrue(swerve.zeroYawCommand());
    
    driverXbox.a()
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1));

    driverXbox.b()
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2));

    driverXbox.x()
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3));

    driverXbox.y()
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L4));
    
    driverXbox.start().onTrue(swerve.zeroYawCommand()); 

    driverXbox.rightTrigger().onTrue(new RemoveAlgaeCommand(swerve, m_elevator, () -> driverXbox.getRightTriggerAxis()));

    driverXbox.rightBumper().onTrue(Commands.sequence(
      algaeSubsystem.intakeUntilStalled(),
      Commands.parallel(
        // algaeSubsystem.stopMotors()
        algaeSubsystem.holdAlgae()
        // new RemoveAlgaeCommand(swerve, algaeSubsystem)
      )
    ));

    driverXbox.leftBumper().onTrue(Commands.sequence(
      algaeSubsystem.startOutake(),
      new WaitCommand(0.5),
      algaeSubsystem.stopMotors()
    ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
