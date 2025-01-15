// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;


/** Constants for the robot. */
public final class Constants {

  /** The robot's maximum angular velocity. */
  public final class Swerve {
  
    public static final double MAX_ANGULAR_VELOCITY = 2.0 * Math.PI; // 2pi rad/sec = 1 rev/sec
    
    /**
     * Constants specific to the swerve modules See
     * https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
     */
    public static final double MAX_SPEED = Units.feetToMeters(15.12);
    public static final double WHEEL_DIAMETER = 4.0; // Inches
    public static final double DRIVE_GEAR_RATIO = 6.55;
    public static final double STEERING_GEAR_RATIO = 10.29;
    public static final double DRIVER_ENCODER_RESOLUTION = 1.0;
    public static final double STEERING_ENCODER_RESOLUTION = 1.0;
  }
  

  public final class Vision {
    public static final String PHOTONCAMERA_NAME_A = "Arducam_A";
    public static final String PHOTONCAMERA_NAME_B = "Arducam_B";
  }

  public static final Matrix<N3, N1> VISION_STDDEV = new Matrix<N3,N1>(N3.instance, N1.instance, new double[] {2,0,2,0,2.5});
}