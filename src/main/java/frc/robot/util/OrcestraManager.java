package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;

public class OrcestraManager {
  private static OrcestraManager single_instance = null;

  private Orchestra orchestra;

  private OrcestraManager() {
    orchestra = new Orchestra();
  }

  public void load(String file) {
    orchestra.loadMusic("giantsteps.chrp");
  }

  public Orchestra getOrchestra() {
    return orchestra;
  }

  public void addInstrument(ParentDevice instrument, int trackNumber) {
    orchestra.addInstrument(instrument, trackNumber);
  }

  public void addInstrument(ParentDevice instrument) {
    orchestra.addInstrument(instrument);
  }

  public static synchronized OrcestraManager getInstance() {
    if (single_instance == null) single_instance = new OrcestraManager();

    return single_instance;
  }
}
