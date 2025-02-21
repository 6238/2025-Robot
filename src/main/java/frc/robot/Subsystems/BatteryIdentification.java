package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BatteryId;
import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Optional;

public class BatteryIdentification extends SubsystemBase {
  SerialPort serialPort;
  Optional<String> serialBatteryId;
  PowerDistribution powerDist;

  // Past here is uninit untill `serialBatteryId != Optional.empty()`

  public BatteryIdentification() {
    serialPort = new SerialPort(115200, Port.kUSB1);
    powerDist = new PowerDistribution(BatteryId.PDH_ID, ModuleType.kRev);
    serialBatteryId = Optional.empty();
    webserverPageHost();
  }

  public Optional<String> tryGetOptionalSerialName() {
    if (serialPort.getBytesReceived() >= 17) {
      return Optional.ofNullable(serialPort.readString());
    } else {
      return Optional.empty();
    }
  }

  @Override
  public void periodic() {
    if (serialBatteryId.isEmpty()) {
      serialBatteryId = tryGetOptionalSerialName();
    }
    if (serialBatteryId.isPresent()) {}
  }

  // https://fms-manual.readthedocs.io/en/latest/fms-whitepaper/fms-whitepaper.html
  // Ports 5800–5810 are recommended for these types of shenanigans
  // Raw dogging webservers with java builtins untill we decide on a library
  // We dont *need* external libs for this but it would be really really nice
  public void webserverPageHost() {
    new Thread(
            () -> {
              try {
                try (ServerSocket server = new ServerSocket(5807)) {
                  System.out.println("BatteryServer Listening: " + server.getLocalSocketAddress());
                  while (true) {
                    Socket client = server.accept();
                    handleRequests(client);
                  }
                }
              } catch (IOException e) {
                e.printStackTrace();
              }
            })
        .start();
  }

  public void handleRequests(Socket client) throws IOException {
    OutputStream out = client.getOutputStream();
    String battery = serialBatteryId.orElse("Battery ID not available");
    String response = "<html> {BATTERY} </html>";
    response = response.replace("{BATTERY}", battery);
    
    out.write(
        ("HTTP/1.1 200 OK\r\n"
                + "Content-Type: text/plain\r\n"
                + "Content-Length: "
                + response.length()
                + "\r\n"
                + "\r\n"
                + response)
            .getBytes());
  }
}
