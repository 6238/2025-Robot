// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.Swerve.MAX_SPEED;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.imu.NavXSwerve;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

@Logged
public class SwerveSubsystem extends SubsystemBase {

  /** Swerve drive object. */
  private final SwerveDrive swerveDrive;

  private Supplier<Matter> elevatorMatter;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory, Supplier<Matter> elevatorMatter) {
    this.elevatorMatter = elevatorMatter;
    // // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    // // The encoder resolution per motor revolution is 1 per motor revolution.
    // double angleConversionFactor =
    // SwerveMath.calculateDegreesPerSteeringRotation(12.8, 1);
    // // Motor conversion factor is (PI * WHEEL DIAMETER) / (GEAR RATIO * ENCODER
    // RESOLUTION).
    // // In this case the wheel diameter is 4 inches.
    // // The gear ratio is 6.75 motor revolutions per wheel rotation.
    // // The encoder resolution per motor revolution is 1 per motor revolution.
    // double driveConversionFactor =
    // SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4),
    // 6.75, 1);
    // System.out.println("\"conversionFactor\": {");
    // System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    // System.out.println("\t\"drive\": " + driveConversionFactor);
    // System.out.println("}");

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being
    // created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      swerveDrive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED);
      swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    } catch (Exception e) {
      DataLogManager.log("EXSITS: " + directory.exists());
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(
        false); // sHeading correction should only be used while controlling the robot via
    // angle.

    setupPathPlanner();
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {

    List<Matter> matter = List.of(Constants.Swerve.CHASSIS, elevatorMatter.get());

    // Translation2d limitedTranslation = SwerveMath.limitVelocity(
    //   translation,
    //   getFieldVelocity(),
    //   getPose(),
    //   Constants.LOOP_TIME,
    //   125,
    //   matter,
    //   swerveDrive.swerveDriveConfiguration
    // );

    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public Command debugDriveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for
    // this kind of control.
    return run(
        () -> {
          Translation2d scaledInputs =
              SwerveMath.scaleTranslation(
                  new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  public Command driveCommandOnce(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for
    // this kind of control.
    return runOnce(
        () -> {
          Translation2d scaledInputs =
              SwerveMath.scaleTranslation(
                  new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  @NotLogged // SwerveDriveKinematics isn't
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Add vision pose (with standard deviation matrix)
   *
   * @param pose
   * @param stdev
   */
  public void addVisionPose(EstimatedRobotPose pose, Matrix<N3, N1> stdev) {
    swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, stdev);
  }

  /**
   * Add vision pose
   *
   * @param pose
   */
  public void addVisionPose(EstimatedRobotPose pose) {
    swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void moveVerySlowly() {
    setChassisSpeeds(new ChassisSpeeds(0, 0, 0.5));
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  private void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public Command zeroYawCommand() {
    return runOnce(
        () -> {
          this.zeroGyro();
        });
  }

  public void setGyroOffset() {
    // swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), new
    // Rotation2d(radians)));
    // swerveDrive.zeroGyro();
    // swerveDrive.setImuOffset(swerveDrive.getPose().getRotation());
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  public double headingCalculate(double targetHeadingAngleRadians) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    double offset = (ally.get() == Alliance.Blue) ? 0.0 : Math.PI;
    return swerveDrive.swerveController.headingCalculate(
        getPose().getRotation().getRadians() + offset, targetHeadingAngleRadians);
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, headingX, headingY, getHeading().getRadians(), MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param rotation2d The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d rotation2d) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, rotation2d.getRadians(), getHeading().getRadians(), MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /** Lock the swerve drive to prevent it from moving. */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /** Setup AutoBuilder for PathPlanner. */
  public void setupPathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a
          // starting pose)
          this::getFieldVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) ->
              drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
          // Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
              // controller for holonomic drive trains
              new PIDConstants(3, 0.0005, 0.05), // Translation PID constants
              new PIDConstants(3, 0.0005, 0.05) // Rotation PID constants
              ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
          );
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder. Try checking version numbers and clean /deploy/* on the rio",
          e.getStackTrace());
    }
  }

  private void resetGyroTo(Rotation2d rot) {
    // if our gyro isn't a navx, this will throw a ClassCastException
    NavXSwerve imu = (NavXSwerve) swerveDrive.getGyro();
    imu.setOffset(new Rotation3d(0, 0, rot.getRadians()));
  }

  /**
   * Reset the IMU to the current measured rotation.
   *
   * @return a Command
   */
  public Command resetGyroCommand() {
    return runOnce(
        () -> {
          this.resetGyroTo(this.getPose().getRotation());
        });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX Heading X to calculate angle of the joystick.
   * @param headingY Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationSpeedX,
      DoubleSupplier translationSpeedY,
      DoubleSupplier rotationSpeed) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(
        () -> {
          Optional<Alliance> ally = DriverStation.getAlliance();
          double sign = 1.0;
          if (ally.isPresent()) {
            sign = (ally.get() == Alliance.Blue) ? -1.0 : 1.0;
          }

          sign *= Constants.FLIP_DIR ? -1.0 : 1.0;

          Translation2d translation =
              new Translation2d(
                  sign * translationSpeedX.getAsDouble(), sign * translationSpeedY.getAsDouble());
          this.drive(translation, rotationSpeed.getAsDouble(), true);
        });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX Heading X to calculate angle of the joystick.
   * @param headingY Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommandRobotRelative(
      DoubleSupplier translationSpeedX,
      DoubleSupplier translationSpeedY,
      DoubleSupplier rotationSpeed) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(
        () -> {
          Translation2d translation =
              new Translation2d(translationSpeedX.getAsDouble(), translationSpeedY.getAsDouble());
          this.drive(translation, rotationSpeed.getAsDouble(), true);
        });
  }

  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return swerveDrive.swerveDrivePoseEstimator.sampleAt(timestampSeconds);
  }

  public Command align(APTarget target) {
    return this.run(
            () -> {
              SmartDashboard.putNumberArray("TARGET_POSE", new double[]{ target.getReference().getMeasureX().baseUnitMagnitude(), target.getReference().getMeasureY().baseUnitMagnitude(), target.getReference().getRotation().getDegrees() });

              ChassisSpeeds robotRelativeSpeeds = this.getRobotVelocity();
              Pose2d pose = this.getPose();

              APResult output = Constants.kAutopilot.calculate(pose, robotRelativeSpeeds, target);

              /* these speeds are field relative */
              double veloX = output.vx().in(MetersPerSecond);
              double veloY = output.vy().in(MetersPerSecond);
              Rotation2d headingReference = output.targetAngle();
              double diff = headingReference.getRadians()-pose.getRotation().getRadians();
              double appliedRot = Math.abs(diff) > Units.degreesToRadians(2) ? (diff * Constants.kP_ROT) : 0;
              
              SmartDashboard.putNumber("currentRot", pose.getRotation().getDegrees());
              SmartDashboard.putNumber("headingTarget", headingReference.getDegrees());
              SmartDashboard.putNumber("sub", diff);
              SmartDashboard.putNumber("appliedRot", appliedRot);

              ChassisSpeeds chassisSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      veloX,
                      veloY,
                      appliedRot, // Assuming the heading is a simple P-controller
                      pose.getRotation());

              // Set the speeds using the YAGSL SwerveDrive object
              swerveDrive.setChassisSpeeds(chassisSpeeds);
        })
        .until(() -> Constants.kAutopilot.atTarget(this.getPose(), target))
        .finallyDo(() -> swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)));
  }
}
