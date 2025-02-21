package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.util.CameraSettings;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import swervelib.math.Matter;

/** Constants for the robot. */
public final class Constants {

  public static final double LOOP_TIME = 0.02;

  public final class IDs {
    public static final int ELEVATOR_LEADER_MOTOR = 50;
    public static final int ELEVATOR_FOLLOWER_MOTOR = 51;
  }

  public final class ControlMapping {
    public static final Axis FORWARD_BACKWARD = Axis.kLeftY;
    public static final Axis LEFT_RIGHT = Axis.kLeftX;
    public static final Axis TURN = Axis.kRightX;

    public static final Button ELEVATOR_L1 = Button.kA;
    public static final Button ELEVATOR_L2 = Button.kB;
    public static final Button ELEVATOR_L3 = Button.kX;
    public static final Button ELEVATOR_L4 = Button.kLeftStick;

    public static final Button MOVE_TO_BARGE_BUTTON = Button.kY;

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
    public static final double TURN_kP = 0.75;
    public static final double TURN_kI = 0;
    public static final double TURN_kD = 0;

    public static final double TURN_THRESHOLD = 3;
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
      public static final double ELEVATOR_MIN_HEIGHT = 1.0;
      public static final double ELEVATOR_MAX_HEIGHT = 68.5;

      public static final double GROUND = 2.0;
      public static final double L1_25 = 10.0;
      public static final double L1_5 = 17.0;
      public static final double L2 = 27.5;
      public static final double L3 = 40;
      public static final double TOP = 68.5; // MAX HEIGHT

      // TODO
      public static final double REACH_STATE_THRES = 0.1;
    }

    public final class DYNAMICS {
      // COM = centre o' mass
      // TODO; these are not even remotely correct
      public static final double TOTAL_MASS = 65.0;
      public static final double COM_HEIGHT_MIN = 0.3;
      public static final double COM_HEIGHT_MAX = 1.3;
      public static final Translation2d COM_LOCATION = new Translation2d();
    }

    // Motion Profile
    public static final double MAX_VELOCITY = 35.0;
    public static final double MAX_ACCEL = 35.0;
    public static final double JERK = 800.0;
  }

  public final class AlgaeEndEffector {
    public static final int LEFT_MOTOR_ID = 40;
    public static final int RIGHT_MOTOR_ID = 41;

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
          "BR_Rico",
          new Transform3d(
              new Translation3d(Inches.of(13.5), Inches.of(-13.5), Inches.of(10.5)),
              new Rotation3d(
                  Degrees.of(0).in(Radians),
                  Degrees.of(-55).in(Radians),
                  Degrees.of(180 + 55).in(Radians)))),
      // new CameraSettings(
      //     "BL_Private",
      //     new Transform3d(
      //         new Translation3d(Inches.of(-13.5), Inches.of(-13.5), Inches.of(10.5)),
      //         new Rotation3d(
      //             Degrees.of(0).in(Radians),
      //             Degrees.of(-55).in(Radians),2
      //             Degrees.of(180 - 55).in(Radians)))),
      new CameraSettings(
          "FR_Kowalski",
          new Transform3d(
              new Translation3d(Inches.of(9.5), Inches.of(6), Inches.of(26.5)),
              new Rotation3d(
                  Degrees.of(0).in(Radians),
                  Degrees.of(0).in(Radians),
                  Degrees.of(270).in(Radians)))),
      new CameraSettings(
          "FL_Skipper",
          new Transform3d(
              new Translation3d(Inches.of(-14.5), Inches.of(0), Inches.of(11.5)),
              new Rotation3d(
                  Degrees.of(0).in(Radians),
                  Degrees.of(65).in(Radians),
                  Degrees.of(90).in(Radians)))),
    };

    public static final String ALGAECAM_NAME = "ALGAE_CAM";
    public static final double VISION_TURN_kP = 0.75;

    public static final Matrix<N3, N1> VISION_STDDEV =
        new Matrix<N3, N1>(N3.instance, N1.instance, new double[] {7, 0, 7, 0, 10});
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
    public static final Pose2d BARGE_TOP = new Pose2d(7.250, 6, Rotation2d.fromDegrees(0));

    public static final Pose2d BARGE_BOTTOM = new Pose2d(7.250, 2, Rotation2d.fromDegrees(0));
  }
}
