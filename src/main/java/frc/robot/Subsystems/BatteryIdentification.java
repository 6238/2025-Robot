package frc.robot.subsystems;

import edu.wpi.first.hal.SerialPortJNI;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BatteryIdentification extends SubsystemBase {
    SerialPort serialPort;
    boolean hasRead = false;

    public BatteryIdentification() {
        serialPort = new SerialPort(115200, Port.kUSB);
        hasRead = false;
    }

    public void startRead() {
        serialPort.writeString("read"); // SEND REQUEST TO READ
    }

    public void readIfAvaliable() {
        if (serialPort.getBytesReceived() >= 17 && !hasRead) { // BATTERY MESSAGE AVALIABLE
            DataLogManager.log("BATTERY: " + serialPort.readString()); // LOG BATTERY NAME
            hasRead = true;
        }
    }

    public void periodic() {
        readIfAvaliable();
    }
}
