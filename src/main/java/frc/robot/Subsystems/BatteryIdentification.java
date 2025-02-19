package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BuildConstants;

public class BatteryIdentification extends SubsystemBase {
  SerialPort serialPort;
  boolean hasRead = false;

  public BatteryIdentification() {
    serialPort = new SerialPort(115200, Port.kUSB1);
    hasRead = false;
  }

  public void readIfAvaliable() {
    if (serialPort.getBytesReceived() >= 17 && !hasRead) { // BATTERY MESSAGE AVALIABLE
      NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/BATTERY_NAME")
        .publish()
        .set(serialPort.readString()); // LOG BATTERY NAME
      hasRead = true;
    }
  }

  public void periodic() {
    readIfAvaliable();
  }
}
