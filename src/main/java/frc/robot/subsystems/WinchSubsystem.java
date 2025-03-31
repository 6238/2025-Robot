// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Winch;
import frc.robot.util.OrcestraManager;

@Logged
public class WinchSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(Winch.MOTOR_ID);
  private final NeutralOut neutralRequest = new NeutralOut();

  public static final double MAX = 175;
  public static final double MIN = -100;

  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {
    AudioConfigs audioConfigs = new AudioConfigs();
    audioConfigs.AllowMusicDurDisable = true;
    motor.getConfigurator().apply(audioConfigs);

    OrcestraManager.getInstance().addInstrument(motor);

    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void raise() {
    if (motor.getPosition().getValueAsDouble() > MIN) {
      motor.setVoltage(-12);
    } else {
      motor.setVoltage(0);
    }
  }

  public void lower() {
    if (motor.getPosition().getValueAsDouble() < MAX) {
      motor.setVoltage(12);
    } else {
      motor.setVoltage(0);
    }
  }

  public void stopMotor() {
    motor.setVoltage(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Winch/motorPosTurns", motor.getPosition().getValueAsDouble());
  }
}
