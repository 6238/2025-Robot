package frc.robot.telemetry;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/**
 * A wrapper for Triggers that implements the Sendable interface, so it can be used for telemetry.
 */
public class SendableTrigger extends Trigger implements Sendable {

  public SendableTrigger(BooleanSupplier condition) {
    super(condition);
  }

  public SendableTrigger(EventLoop loop, BooleanSupplier condition) {
    super(loop, condition);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Trigger");
    builder.addBooleanProperty(
        "value",
        this::getAsBoolean,
        null); // SendableBuilder is null-safe (checked source) --ajs 2024-08-16
  }
}
