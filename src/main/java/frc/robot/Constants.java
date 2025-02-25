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
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.util.CameraSettings;
import java.io.File;
import java.util.Map;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import swervelib.math.Matter;

/** Constants for the robot. */
public final class Constants {

  public static final double LOOP_TIME = 0.02;

  public static final Map<String, File> SWERVE_DIRECTORIES =
      Map.ofEntries(
          entry("032B1F73", new File(Filesystem.getDeployDirectory(), "swerve2")),
          entry("03182373", new File(Filesystem.getDeployDirectory(), "swerve")));

  public final class IDs {
    public static final int ELEVATOR_LEADER_MOTOR = 50;
    public static final int ELEVATOR_FOLLOWER_MOTOR = 51;
  }

  public final class ControlMapping {
    public static final Axis FORWARD_BACKWARD = Axis.kLeftY;
    public static final Axis LEFT_RIGHT = Axis.kLeftX;
    public static final Axis TURN = Axis.kRightX;

    public static final Button GROUND = Button.kA;
    public static final Button MOVE_TO_BARGE = Button.kX;
    public static final Button LIFT_TO_REEF = Button.kB;
    public static final Button CHASE_CORAL = Button.kY;

    public static final Axis MOVE_TO_BARGE_AXIS = Axis.kLeftTrigger;
    public static final double MOVE_TO_BARGE_THRESHOLD = 0.5;

    public static final Button OUTTAKE = Button.kRightBumper;
    public static final Button INTAKE = Button.kLeftBumper;

    public static final Axis ELEVATOR_RAISE_LOWER = Axis.kRightY;
    public static final double ELEVATOR_ADJUST_THRESHOLD = 0.25;
    public static final double ELEVATOR_ADJUST_SPEED_DECREASE = 50;
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
      public static final double kS = 0.041645; // voltage to overcome static friction
      public static final double kG = 0.42517; // voltage to overcome gravity
      public static final double kV = 0.12811; // volts per 1 rps
      public static final double kA = 0.0060141; // volts per 1 rps/s

      public static final double kg_Ball = 0.465;
      public static final double kg_Top = 0.6;

      // PID for correcting errors
      public static final double kP = 5;
      public static final double kI = 0.05;
      public static final double kD = 0;
    }

    public final class ElevatorHeights {
      public static final double ELEVATOR_GEAR_RATIO = (45.1236) / 80;

      // Min and Max Height for the Elevator
      public static final double ELEVATOR_MIN_HEIGHT = 4;
      public static final double ELEVATOR_MAX_HEIGHT = 72.25;

      public static final double GROUND = 9.0;
      public static final double L1_25 = 14.0;
      public static final double L1_5 = 20.0;
      public static final double L2 = 31.5;
      public static final double L3 = 45;
      public static final double TOP = 72.25; // MAX HEIGHT

      public static final double REACH_STATE_THRES = 0.1;
    }

    public final class DYNAMICS {
      // COM = centre o' mass
      public static final double TOTAL_MASS = Mass.ofBaseUnits(30, Pounds).in(Kilogram);
      public static final Translation2d COM_LOCATION = new Translation2d(0, 6);
    }

    // Motion Profile
    public static final double MAX_VELOCITY = 20.0;
    public static final double MAX_ACCEL = 30.0;
    public static final double JERK = 800.0;
  }

  public final class AlgaeEndEffector {
    public static final int LEFT_MOTOR_ID = 41;
    public static final int RIGHT_MOTOR_ID = 40;

    public static final double STALL_THRESHOLD = 0.1;

    public static final double INTAKE_SPEED = 32.5;

    public static final double OUTAKE_WAIT = 3.0;

    public static final double REEF_REMOVAL_SPEED = 0.65;
    public static final Distance REEF_REMOVAL_DIST = Inches.of(12);
    public static final double REEF_REMOVAL_CONTROLLER_VAL = 0.75;
  }

  public final class Vision {
    public static final PoseStrategy VISION_POSE_STRATEGY =
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static final CameraSettings CAMERA_SETTINGS[] = {
      new CameraSettings(
          "BR",
          new Transform3d(
              new Translation3d(Inches.of(-10.5), Inches.of(-10.5), Inches.of(10)),
              new Rotation3d(
                  Degrees.of(0).in(Radians),
                  Degrees.of(0).in(Radians),
                  Degrees.of(180 + 45).in(Radians)))),
      new CameraSettings(
          "BL",
          new Transform3d(
              new Translation3d(Inches.of(-10.5), Inches.of(10.5), Inches.of(10)),
              new Rotation3d(
                  Degrees.of(0).in(Radians),
                  Degrees.of(0).in(Radians),
                  Degrees.of(180 - 45).in(Radians)))),
      new CameraSettings(
          "FR",
          new Transform3d(
              new Translation3d(Inches.of(3.5), Inches.of(-14.5), Inches.of(15.5)),
              new Rotation3d(
                  Degrees.of(0).in(Radians),
                  Degrees.of(0).in(Radians),
                  Degrees.of(270).in(Radians)))),
    };

    public static final String ALGAECAM_NAME = "AlgaeCam";

    public static final Matrix<N3, N1> VISION_STDDEV =
        new Matrix<N3, N1>(N3.instance, N1.instance, new double[] {12, 0, 12, 0, 16});
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
    public static final double BARGE_X_BLUE = 7.5;
    public static final double BARGE_X_RED = 10.5;

    public static final double BARGE_SPEED = 0.8;
    public static final double BARGE_THRESHOLD = 0.2;

    public static final Pose2d REEF_CENTER_BLUE = new Pose2d(4.485, 4.042, new Rotation2d());

    public static final Pose2d REEF_CENTER_RED = new Pose2d(4.485, 4.042, new Rotation2d());
  }

  public class PathfindingConfig {
    public static final double DRIVE_RESUME_DEADBAND = 0.05;

    /*
     * Values were from path planner
     * X and Y are in Meters
     * Rotation is Rotation2d
     *
     * Each Point has a Pose2d and a GoalEndState
     */
    public static final Pose2d BARGE_BLUE = new Pose2d(7.250, 6, Rotation2d.fromDegrees(0));
    public static final Pose2d BARGE_BLUE_FLIPPED =
        new Pose2d(10.250, 6, Rotation2d.fromDegrees(0));
    public static final Pose2d BARGE_RED = new Pose2d(7.250, 2, Rotation2d.fromDegrees(0));
    public static final Pose2d BARGE_RED_FLIPPED = new Pose2d(7.250, 2, Rotation2d.fromDegrees(0));
  }
}
