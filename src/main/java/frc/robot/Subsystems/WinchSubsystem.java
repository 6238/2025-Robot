// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static java.util.Map.entry;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Winch;
import java.util.Map;

public class WinchSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(Winch.MOTOR_ID);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private enum Position {
    /** Initial state */
    UNKNOWN,
    /** Fully retracted to lift the robot */
    PULL,
    /** Middle position - won't interfere with elevator */
    STOW,
    /** Fully outward to grab onto cage */
    GRAB
  }

  private Map<Position, Double> POSITIONS =
      Map.ofEntries(
          entry(Position.GRAB, 0.0),
          entry(Position.STOW, 10.0),
          entry(Position.PULL, 20.0)); // TODO: these are bogus and should be set

  private Position currentPosition = Position.UNKNOWN;

  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.setControl(neutralRequest);
    var slotConfigs = new Slot0Configs();
    slotConfigs.kP = Winch.Gains.kP;
    slotConfigs.kI = Winch.Gains.kI;
    slotConfigs.kD = Winch.Gains.kD;
    motor.getConfigurator().apply(slotConfigs);
    isAtGrabPosition()
        .onTrue(
            Commands.runOnce(
                () -> {
                  motor.setPosition(0);
                  currentPosition = Position.GRAB;
                }));
  }

  /** Sends the arm to the outward ("grab") position. */
  public Command toGrab() {
    return startEnd(
            () -> {
              motor.setControl(positionRequest.withPosition(POSITIONS.get(Position.GRAB)));
            },
            () -> {
              motor.setControl(neutralRequest);
            })
        .until(this.isAtGrabPosition());
  }

  /** Sends the arm to the stowed position. */
  public Command toStow() {
    return startEnd(
            () -> {
              motor.setControl(positionRequest.withPosition(POSITIONS.get(Position.STOW)));
            },
            () -> {
              motor.setControl(neutralRequest);
            })
        .until(this.isAtStowPosition());
  }

  public Command toPull() {
    return startEnd(
            () -> {
              motor.setControl(positionRequest.withPosition(POSITIONS.get(Position.PULL)));
            },
            () -> {
              motor.setControl(neutralRequest);
            })
        .until(this.isAtPullPosition());
  }

  private boolean limitTriggered() {
    return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  private boolean isAtPosition(Position pos) {
    return Math.abs(motor.getPosition().getValueAsDouble() - POSITIONS.get(pos)) <= Winch.TOLERANCE;
  }

  public Trigger isAtStowPosition() {
    return new Trigger(() -> this.isAtPosition(Position.STOW));
  }

  public Trigger isAtPullPosition() {
    return new Trigger(() -> this.isAtPosition(Position.PULL));
  }

  public Trigger isAtGrabPosition() {
    return new Trigger(this::limitTriggered);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Winch/motorPosTurns", motor.getPosition().getValueAsDouble());
  }
}
