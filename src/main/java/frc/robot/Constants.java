// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Map;


/** Constants for the robot. */
public final class Constants {

  public final class IDs {
    public static final int INTAKE_MOTOR = 41;
    public static final int OUTTAKE_TOP_MOTOR = 42;
    public static final int OUTTAKE_BOTTOM_MOTOR = 43;
    public static final int ANGLE_MOTOR = 62;
    public static final int ROLLER_MOTOR = 61;
  }

  /** The robot's maximum angular velocity. */
  public static final double MAX_ANGULAR_VELOCITY = 2.0 * Math.PI; // 2pi rad/sec = 1 rev/sec

  /** Constants for the {@link frc.robot.subsystems.ArmSubsystem}. */
  public final class Arm {
    /**
     * Position of the absolute encoder (in ticks, i.e. 1/4096 of a turn) that represents angle 0.
     */
    public static final double ENCODER_ZERO = 3212;

    /** Number of arm rotations that one full motor rotation produces. */
    public static final double ARM_TO_MOTOR_RATIO = 0.238480315;

    /** PID gains for moving the arm. */
    public final class Gains {
      public static final double kP = 1.0;
      public static final double kI = 0.05;
      public static final double kD = 0.001;
    }

    /** States the arm can be in */
    public enum ArmStates {
      TRANSFER,
      /** Lowered for intaking */
      INTAKE,
      /** Stowed - entirely within frame */
      STOW,
      /** Raised to shoot in speaker */
      SHOOT
    }

    public static final Map<ArmStates, Double> ANGLES =
        Map.ofEntries(
            entry(ArmStates.TRANSFER, 60.0),
            entry(ArmStates.INTAKE, 22.0),
            entry(ArmStates.SHOOT, 45.0),
            entry(ArmStates.STOW, 85.0));
  }

  public final class Vision {
    public static final String PHOTONCAMERA_NAME_A = "Arducam_A";
    public static final String PHOTONCAMERA_NAME_B = "Arducam_B";
  }

  public static class OuttakeGains {
    public static final double kP = 1e-4;
    public static final double kI = 1e-7;
    public static final double kD = 1e-5;
    public static final double kIz = 0;
    public static final double kFF = 0.000135;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700.0;
  }

  public static class Speeds {
    public static final double INTAKE_SPEED = -0.55;
    public static final double OUTTAKE_SPEED = 3000;
    public static final double SHOOTER_TRANSFER_SPEED = 350;
    public static final double AMP_TRANSFER_SPEED = -.35;
  }

  public static final Matrix<N3, N1> VISION_STDDEV = new Matrix<N3,N1>(N3.instance, N1.instance, new double[] {2,0,2,0,2.5});
}