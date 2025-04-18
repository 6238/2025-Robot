package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import static java.util.Map.entry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.CameraSettings;

import java.io.File;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.util.DrivingRate;
import frc.robot.util.DrivingRate.DrivingRateConfig;
import swervelib.math.Matter;

/** Constants for the robot. */
public final class Constants {

  public static boolean FLIP_DIR = true;

  public static final double LOOP_TIME = 0.02;

  public static final Map<String, File> SWERVE_DIRECTORIES =
      Map.ofEntries(
          entry("032B1F73", new File(Filesystem.getDeployDirectory(), "swerve2")),
          entry("03182373", new File(Filesystem.getDeployDirectory(), "swerve")));

  public final class IDs {
    public static final int ELEVATOR_LEADER_MOTOR = 50;
    public static final int ELEVATOR_FOLLOWER_MOTOR = 51;
  }

  /** The robot's maximum angular velocity. */
  public final class Swerve {
    public static final Matter CHASSIS = new Matter(new Translation3d(0.0, 0.0, 0.1), 60.0);
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

    /* Turn PID */
    public static final double TURN_kP = 0.175;
    public static final double TURN_kI = 0.005;
    public static final double TURN_kD = 0;

    public static final double TURN_THRESHOLD = 10;
  }

  public final class Elevator {
    public final class Gains {
      // From SysID routine
      public static double kS = 0.041645; // voltage to overcome static friction
      public static double kG = 0.38; // voltage to overcome gravity
      public static double kV =
          8.84 * Units.inchesToMeters(ElevatorHeights.ELEVATOR_GEAR_RATIO); // volts per 1 rps
      public static double kA =
          0.13 * Units.inchesToMeters(ElevatorHeights.ELEVATOR_GEAR_RATIO); // volts per 1 rps/s

      public static double kg_Ball = 0.465;
      public static double kg_Top = 2;

      // PID for correcting errors
      public static double kP = 3;
      public static double kI = 0.05;
      public static double kD = 0;
    }

    public final class ElevatorHeights {
      public static final double ELEVATOR_GEAR_RATIO = (42.6) / 80.5;

      // Min and Max Height for the Elevator
      public static final double ELEVATOR_MIN_HEIGHT = -0.2;
      public static final double ELEVATOR_MAX_HEIGHT = 81.25;

      public static final double STOW = 0;
      public static final double GROUND = 7;
      public static final double L1_25 = 13.5-1.5;
      public static final double L1_5 = 18.5-1.5;
      public static final double L2 = 34.5-1.5;
      public static final double L3 = 48.5;
      public static final double TOP = 80; // MAX HEIGHT

      // TODO
      public static final double REACH_STATE_THRES = 0.1;
    }

    public final class DYNAMICS {
      // COM = centre o' mass
      public static final double TOTAL_MASS = Mass.ofBaseUnits(30, Pounds).in(Kilogram);
      public static final Translation2d COM_LOCATION = new Translation2d(0, 6);
    }

    // Motion Profile
    public static double MAX_VELOCITY = 50.0;
    public static double MAX_ACCEL = 60.0;
    public static double MAX_JERK = 1800.0;

    public static double ELEVATOR_OFFSET = 0;
  }

  public final class AlgaeEndEffector {
    public static final int LEFT_MOTOR_ID = 40;
    public static final int RIGHT_MOTOR_ID = 41;

    public static final double STALL_THRESHOLD = 0.1;

    public static double INTAKE_SPEED = 80;
    public static double OUTAKE_SPEED = 0.04;

    public static final double OUTAKE_WAIT = 3.0;

    public static final double REEF_REMOVAL_SPEED = 0.65;
    public static final Distance REEF_REMOVAL_DIST = Inches.of(12);
    public static final double REEF_REMOVAL_CONTROLLER_VAL = 0.75;
  }

  public final class Vision {
    public static final PoseStrategy VISION_POSE_STRATEGY =
        PoseStrategy.LOWEST_AMBIGUITY;

    public static final CameraSettings[] CAMERA_SETTINGS = {
      new CameraSettings(
          "Reef_L",
          new Transform3d(
              new Translation3d(Inches.of(12.615295), Inches.of(10.696954), Inches.of(7.612962+1.4785)),
              new Rotation3d(
                  Degrees.of(0).in(Radians),
                  Degrees.of(-10).in(Radians),
                  Degrees.of(-20).in(Radians)))),
      new CameraSettings(
        "Reef_R",
        new Transform3d(
            new Translation3d(Inches.of(12.615), Inches.of(-10.841124), Inches.of(7.612962+1.4785)),
            new Rotation3d(
                Degrees.of(0).in(Radians),
                Degrees.of(-10).in(Radians),
                Degrees.of(20).in(Radians)))),
      new CameraSettings(
          "BR",
          new Transform3d(
              new Translation3d(Inches.of(-12.306612), Inches.of(-12.743715), Inches.of(7.598246+1.875)),
              new Rotation3d(
                  Degrees.of(0).in(Radians),
                  Degrees.of(-10).in(Radians),
                  Degrees.of(180+45).in(Radians)))),
      new CameraSettings(
        "BL",
        new Transform3d(
            new Translation3d(Inches.of(-12.306612), Inches.of(12.743715), Inches.of(7.598246+2-0.4)),
            new Rotation3d(
                Degrees.of(0).in(Radians),
                Degrees.of(-10).in(Radians),
                Degrees.of(180 - 45).in(Radians)))),
    };

    public static boolean ENABLE = true;

    public static final double CLOSE_FAR_CUTOFF = Units.feetToMeters(9);
    public static final double AMBIGUITY_CUTOFF = 0.2;

    public static final Matrix<N3, N1> REEF_CLOSE_VISION_STDDEV =
      new Matrix<N3, N1>(N3.instance, N1.instance, new double[] {0.01, 0.01, Math.toRadians(5)});
    
    public static final Matrix<N3, N1> REEF_FAR_VISION_STDDEV =
      new Matrix<N3, N1>(N3.instance, N1.instance, new double[] {0.05, 0.05, Math.toRadians(15)});
  }

  public final class Winch {
    public static final int MOTOR_ID = 60;
    public static final double TARGET_VOLTAGE = 10.0;
    public static final double TOLERANCE = 0.1;

    public final class Gains {
      public static final double kP = 2.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }
  }

  public final class AutoMoveGeneration {
    public static final double BARGE_X_BLUE = 7.1;
    public static final double BARGE_X_RED = 10.75;

    public static final double BARGE_SPEED = 0.8;
    public static final double BARGE_THRESHOLD = 0.2;
  }

  public class PathfindingConfig {
    public static final double DRIVE_RESUME_DEADBAND = 0.2;

    /*
     * Values were from path planner
     * X and Y are in Meters
     * Rotation is Rotation2d
     *
     * Each Point has a Pose2d and a GoalEndState
     */
    public static final Pose2d BARGE_BLUE = new Pose2d(7.10, 5.65, Rotation2d.fromDegrees(25));
    public static final Pose2d BARGE_BLUE_FLIPPED =
        new Pose2d(10.35, 6, Rotation2d.fromDegrees(180-25));
    public static final Pose2d BARGE_RED = new Pose2d(10.35, 2, Rotation2d.fromDegrees(180-25));
    public static final Pose2d BARGE_RED_FLIPPED = new Pose2d(7.1, 2, Rotation2d.fromDegrees(25));

    public static final Pose2d BLUE_REEF_CENTER = new Pose2d(4.486, 4.027, Rotation2d.kZero);
    public static final Pose2d RED_REEF_CENTER = new Pose2d(13.062, 4.027, Rotation2d.kZero);
  }
}
