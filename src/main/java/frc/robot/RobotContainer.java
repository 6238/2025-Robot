// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class RobotContainer {
    XboxController m_driverController = new XboxController(0);
    private ElevatorSubsystem m_elevator = new ElevatorSubsystem();


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1));

    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2));

    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3));

    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L4));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
