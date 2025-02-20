package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Elevator.ElevatorHeights;
import frc.robot.Constants.Elevator.Gains;
import frc.robot.Constants.IDs;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX leaderMotor;
  private TalonFX followerMotor;

  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  final DigitalInput limit = new DigitalInput(0);

  private NeutralModeValue neutralModeValue = NeutralModeValue.Brake;

  private BooleanSupplier hasBall;

  public ElevatorSubsystem(BooleanSupplier hasBall) {
    leaderMotor = new TalonFX(IDs.ELEVATOR_LEADER_MOTOR);
    followerMotor = new TalonFX(IDs.ELEVATOR_FOLLOWER_MOTOR);

    this.hasBall = hasBall;

    var elevatorMotorConfigs = new TalonFXConfiguration();

    Slot0Configs motorConfig = elevatorMotorConfigs.Slot0;

    elevatorMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.kS = Gains.kS;
    motorConfig.kG = Gains.kG;
    motorConfig.kV = Gains.kV;
    motorConfig.kA = Gains.kA;
    motorConfig.kP = Gains.kP;
    motorConfig.kI = Gains.kI;
    motorConfig.kD = Gains.kD;

    var motionMagicConfigs = elevatorMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Elevator.MAX_VELOCITY; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        Elevator.MAX_ACCEL; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = Elevator.JERK;

    leaderMotor.getConfigurator().apply(elevatorMotorConfigs);
    followerMotor.setControl(new Follower(IDs.ELEVATOR_LEADER_MOTOR, false));

    leaderMotor.setNeutralMode(neutralModeValue);
    followerMotor.setNeutralMode(neutralModeValue);

    this.setHeight(ElevatorHeights.ELEVATOR_MIN_HEIGHT);
  }

  public Command setHeightCommand(double givenHeight) {
    return run(() -> setHeight(givenHeight));
  }

  public void brake() {
    leaderMotor.setNeutralMode(NeutralModeValue.Brake);
    followerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getHeight() {
    return goal.position / ElevatorHeights.ELEVATOR_GEAR_RATIO;
  }

  //// sets the height to a clamped value
  public void setHeight(Double height) {
    double min = ElevatorHeights.ELEVATOR_MIN_HEIGHT;
    double max = ElevatorHeights.ELEVATOR_MAX_HEIGHT;
    goal.position = Math.max(min, Math.min(height, max)) * ElevatorHeights.ELEVATOR_GEAR_RATIO;
  }

  public Command increaseHeight(DoubleSupplier speed) {
    return runOnce(
        () -> {
          goal.position += goal.position + speed.getAsDouble();
        });
  }

  public void resetEncoder() {
    leaderMotor.setPosition(0);
    followerMotor.setPosition(0);
  }

  public void toggleBrakeMode() {
    if (neutralModeValue == NeutralModeValue.Coast) {
      neutralModeValue = NeutralModeValue.Brake;
      leaderMotor.setNeutralMode(NeutralModeValue.Brake);
      followerMotor.setNeutralMode(NeutralModeValue.Brake);
      return;
    }
    neutralModeValue = NeutralModeValue.Coast;
    leaderMotor.setNeutralMode(NeutralModeValue.Coast);
    followerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    if (limit.get() && leaderMotor.getPosition().getValueAsDouble() != 0) {
      resetEncoder();
    }

    if (Math.abs(leaderMotor.getPosition().getValueAsDouble() - goal.position) < 0.15) {
      if (goal.position / ElevatorHeights.ELEVATOR_GEAR_RATIO > 60) {
        leaderMotor.setVoltage(Elevator.Gains.kg_Top);
      } else if (hasBall.getAsBoolean()) {
        leaderMotor.setVoltage(Elevator.Gains.kg_Ball);
      } else {
        leaderMotor.setVoltage(Elevator.Gains.kG);
      }
    } else {
      leaderMotor.setControl(
          m_request.withPosition(goal.position).withLimitReverseMotion(limit.get()));
    }
    SmartDashboard.putNumber(
        "elevator height",
        leaderMotor.getPosition().getValueAsDouble() / ElevatorHeights.ELEVATOR_GEAR_RATIO);
    SmartDashboard.putNumber(
        "elevator setpoint", goal.position / ElevatorHeights.ELEVATOR_GEAR_RATIO);
    SmartDashboard.putNumber("leader_motor", leaderMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("follower_motor", followerMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "leader_motor_voltage", leaderMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "follower_motor_voltage", followerMotor.getMotorVoltage().getValueAsDouble());
  }

  public boolean reachedState() {
    double error = leaderMotor.getPosition().getValueAsDouble() - setpoint.position;
    return Math.abs(error) < ElevatorHeights.REACH_STATE_THRES;
  }

  /* SYSID */
  private Time timeout = Time.ofRelativeUnits(10, Seconds);
  private Velocity<VoltageUnit> ramp = Volts.per(Seconds).ofBaseUnits(0.5);

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              ramp, // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduce dynamic step voltage to 4 to prevent brownout
              timeout, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                leaderMotor.setControl(m_voltReq.withOutput(volts.in(Volts)));
                followerMotor.setControl(m_voltReq.withOutput(volts.in(Volts)));
              },
              null,
              this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
