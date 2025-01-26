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
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.CameraSettings;


/** Constants for the robot. */
public final class Constants {
  
  public final class IDs {
    public static final int ELEVATOR_MOTOR = 30; // TODO: Determine Elevator Motor ID
  }

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
  
  public final class Elevator {
    public final class Gains {
      // From SysID routine
      public static final double kS = 0.16002; // voltage to overcome static friction
      public static final double kG =-0.19416; // voltage to overcome gravity
      public static final double kV = 0.10967; // volts per 1 rps
      public static final double kA = 0.001425; // volts per 1 rps/s

      // PID for correcting errors
      public static final double kP = 0.75;
      public static final double kI = 0.02;
      public static final double kD = 0.0;
    }

    public final class ElevatorHeights {
      public static final double ELEVATOR_GEAR_RATIO = 1.62976;

      // Min and Max Height for the Elevator
      public static final double ELEVATOR_MIN_HEIGHT = 0.0;
      public static final double ELEVATOR_MAX_HEIGHT = 60.0;

      //TODO: find heights
      public static final double L1 = 1.0;
      public static final double L2 = 10.0;
      public static final double L3 = 28.2;
      public static final double L4 = 44.5;

      // TODO
      public static final double REACH_STATE_THRES = 0.1;
    }

    // Motion Profile
    public static final double MAX_VELOCITY = 25.0;
    public static final double MAX_ACCEL = 50.0;
    public static final double JERK = 500.0;
  }

  public final class AlgaeEndEffector {
    public static final int LEFT_MOTOR_ID = 40;
    public static final int RIGHT_MOTOR_ID = 41;

    public static final double STALL_THRESHOLD = 0.1;

    public static final double INTAKE_SPEED = 25;
    public static final double OUTAKE_SPEED = 60;

    public static final double OUTAKE_WAIT = 3.0;

    public static final double REEF_REMOVAL_SPEED = 1;
    public static final Distance REEF_REMOVAL_DIST = Inches.of(12);
    public static final double REEF_REMOVAL_CONTROLLER_VAL = 0.75;
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