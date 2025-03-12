// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static java.util.Map.entry;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Winch;
import frc.robot.util.OrcestraManager;
import java.util.Map;

@Logged
public class WinchSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(Winch.MOTOR_ID);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public static enum Position {
    /** Initial state */
    UNKNOWN,
    /** Fully retracted to lift the robot */
    PULL,
    /** Fully outward to grab onto cage */
    GRAB
  }

  public static final Map<Position, Double> POSITIONS =
      Map.ofEntries(entry(Position.GRAB, 0.0), entry(Position.PULL, -200.0));

  private Position currentPosition = Position.UNKNOWN;

  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {
    AudioConfigs audioConfigs = new AudioConfigs();
    audioConfigs.AllowMusicDurDisable = true;
    motor.getConfigurator().apply(audioConfigs);

    OrcestraManager.getInstance().addInstrument(motor);

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

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void stopMotor() {
    motor.setVoltage(0);
  }

  private boolean limitTriggered() {
    return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  private boolean isAtPosition(Position pos) {
    return Math.abs(motor.getPosition().getValueAsDouble() - POSITIONS.get(pos)) <= Winch.TOLERANCE;
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
