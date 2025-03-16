// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static java.util.Map.entry;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;

@Logged
public class LEDSubsystem extends SubsystemBase {
  private CANdle candle = new CANdle(40);
  private final int LED_COUNT = 62; // Just the onboard LEDs
  private LEDMode currentMode = null;
  private LEDMode lastSentMode = null;

  public LEDSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = false;
    config.brightnessScalar = 0.7;
    config.vBatOutputMode = VBatOutputMode.Off;
    candle.configAllSettings(config);

    currentMode = LEDMode.RAINBOW;
  }

  @Override
  public void periodic() {
    if (currentMode != lastSentMode) {
      candle.animate(MODES.get(currentMode));
      lastSentMode = currentMode;
    }

    // If disabled, we want to do the alliance pulse or default to rainbow
    if (DriverStation.isDisabled()) {
      this.setAnimation(this.getDefaultAnimation(DriverStation.getAlliance()));
    }
  }

  public void setAnimation(LEDMode mode) {
    currentMode = mode;
  }

  private LEDMode getDefaultAnimation(Optional<Alliance> ally) {
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        return DriverStation.isEnabled() ? LEDMode.RED_FAST_PULSE : LEDMode.RED_SLOW_PULSE;
      } else {
        return DriverStation.isEnabled() ? LEDMode.BLUE_FAST_PULSE : LEDMode.BLUE_SLOW_PULSE;
      }
    } else {
      return LEDMode.RAINBOW;
    }
  }

  public Command setAnimationToAllianceColorCommand(Optional<Alliance> ally) {
    return runOnce(
            () -> {
              LEDMode anim = getDefaultAnimation(ally);
              currentMode = anim;
            })
        .ignoringDisable(true);
  }

  public Command indicateIntookCommand() {
    return runOnce(
        () -> {
          currentMode = LEDMode.YELLOW_CHASE;
        });
  }

  public Command climbCommand() {
    return runOnce(
      () -> {
        currentMode = LEDMode.DAS_FIRE;
      });
  }

  public enum LEDMode {
    RED_SLOW_PULSE,
    BLUE_SLOW_PULSE,
    RED_FAST_PULSE,
    BLUE_FAST_PULSE,
    RAINBOW,
    CYAN_CHASE,
    YELLOW_CHASE,
    SCANNER,
    DAS_FIRE,
    OFF
  }

  // spotless:off
  private final EnumMap<LEDMode, Animation> MODES =
      new EnumMap<LEDMode, Animation>(Map.ofEntries(
            entry(LEDMode.OFF,   new SingleFadeAnimation(0, 0, 0, 0, 0.5, LED_COUNT, 8)),
            entry(LEDMode.RED_SLOW_PULSE,   new SingleFadeAnimation(255, 0, 0, 0, 0.5, LED_COUNT, 8)),
            entry(LEDMode.BLUE_SLOW_PULSE,  new SingleFadeAnimation(0, 0, 255, 0, 0.5, LED_COUNT, 8)),
            entry(LEDMode.RED_FAST_PULSE,   new SingleFadeAnimation(255, 0, 0, 0, 0.8, LED_COUNT, 8)),
            entry(LEDMode.BLUE_FAST_PULSE,  new SingleFadeAnimation(0, 0, 255, 0, 0.8, LED_COUNT, 8)),
            entry(LEDMode.RAINBOW,          new RainbowAnimation(1.0, 0.5, LED_COUNT, false, 8)),
            entry(LEDMode.CYAN_CHASE,       new ColorFlowAnimation(10, 250, 182, 0, 0.8, LED_COUNT, Direction.Backward, 8)),
            entry(LEDMode.YELLOW_CHASE,     new ColorFlowAnimation(255, 191, 0, 0, 0.8, LED_COUNT, Direction.Backward, 8)),
            entry(LEDMode.SCANNER,          new LarsonAnimation(255, 191, 0, 0, 0.8, LED_COUNT, BounceMode.Back, 7, 8)),
            entry(LEDMode.DAS_FIRE,         new FireAnimation(0.75, 0.7, LED_COUNT, 0.7, 0.2))
            ));
  // spotless:on
}
