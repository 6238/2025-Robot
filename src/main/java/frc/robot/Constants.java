// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.CameraSettings;


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
    public static final PoseStrategy VISION_POSE_STRATEGY = PoseStrategy.AVERAGE_BEST_TARGETS;

    public static final CameraSettings CAMERA_SETTINGS[] = {
      new CameraSettings(
        "Arducam_A",
        new Transform3d(
          new Translation3d(
            Inches.of(-13.5),
            Inches.of(13.5),
            Inches.of(10.5)
          ),
          new Rotation3d(
            Degrees.of(0).in(Radians),
            Degrees.of(-55).in(Radians),
            Degrees.of(150).in(Radians)
          )
        )
      ),
      new CameraSettings(
        "Arducam_B",
        new Transform3d(
          new Translation3d(
            Inches.of(-13.5),
            Inches.of(-13.5),
            Inches.of(10.5)
          ),
          new Rotation3d(
            Degrees.of(180).in(Radians),
            Degrees.of(-55).in(Radians),
            Degrees.of(210).in(Radians)
          )
        )
      )
    };
    
    public static final Matrix<N3, N1> VISION_STDDEV = new Matrix<N3,N1>(N3.instance, N1.instance, new double[] {2,0,2,0,2.5});
  }
}