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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.Constants.Elevator.DYNAMICS;
import frc.robot.Constants.Elevator.ElevatorHeights;
import frc.robot.Constants.Elevator.Gains;
import frc.robot.Constants.IDs;
import frc.robot.util.OrcestraManager;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.math.Matter;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX leaderMotor;
  private TalonFX followerMotor;

  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  final DigitalInput limit = new DigitalInput(0);

  private NeutralModeValue neutralModeValue = NeutralModeValue.Brake;

  private BooleanSupplier hasBall;
  private int slotNum = 0;

  public ElevatorSubsystem(BooleanSupplier hasBall) {
    leaderMotor = new TalonFX(IDs.ELEVATOR_LEADER_MOTOR, "canivore");
    followerMotor = new TalonFX(IDs.ELEVATOR_FOLLOWER_MOTOR, "canivore");

    this.hasBall = hasBall;

    var elevatorMotorConfigs = new TalonFXConfiguration();
    var fastElevatorMotorConfigs = new TalonFXConfiguration();

    Slot0Configs motorConfig = elevatorMotorConfigs.Slot0;

    motorConfig.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.kS = Gains.kS;
    motorConfig.kG = Gains.kG;
    motorConfig.kV = Gains.kV;
    motorConfig.kA = Gains.kA;
    motorConfig.kP = Gains.kP;
    motorConfig.kI = Gains.kI;
    motorConfig.kD = Gains.kD;

    elevatorMotorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        Elevator.MAX_VELOCITY; // Target cruise velocity of 80 rps
    elevatorMotorConfigs.MotionMagic.MotionMagicAcceleration =
        Elevator.MAX_ACCEL; // Target acceleration of 160 rps/s (0.5 seconds)
    elevatorMotorConfigs.MotionMagic.MotionMagicJerk = Elevator.JERK;
    leaderMotor.getConfigurator().apply(elevatorMotorConfigs);

    followerMotor.setControl(new Follower(IDs.ELEVATOR_LEADER_MOTOR, true));

    leaderMotor.setNeutralMode(neutralModeValue);
    followerMotor.setNeutralMode(neutralModeValue);

    OrcestraManager.getInstance().addInstrument(leaderMotor);
    OrcestraManager.getInstance().addInstrument(followerMotor);

    this.setHeight(ElevatorHeights.ELEVATOR_MIN_HEIGHT);
  }

  public Command setHeightCommand(double givenHeight) {
    return runOnce(() -> setHeight(givenHeight));
  }

  public double getTargetHeight() {
    return goal.position / ElevatorHeights.ELEVATOR_GEAR_RATIO;
  }

  public void brake() {
    leaderMotor.setNeutralMode(NeutralModeValue.Brake);
    followerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getHeight() {
    return leaderMotor.getPosition().getValueAsDouble() / ElevatorHeights.ELEVATOR_GEAR_RATIO;
  }

  public double getVerticalAcceleration() {
    return leaderMotor.getAcceleration().getValueAsDouble() / ElevatorHeights.ELEVATOR_GEAR_RATIO;
  }

  //// sets the height to a clamped value
  public void setHeight(Double height) {
    double min = ElevatorHeights.ELEVATOR_MIN_HEIGHT;
    double max = ElevatorHeights.ELEVATOR_MAX_HEIGHT;
    goal.position = Math.max(min, Math.min(height, max)) * ElevatorHeights.ELEVATOR_GEAR_RATIO;
    // if (height < getHeight()) {
    //   slotNum = 1;
    // } else {
    //   slotNum = 0;
    // }
  }

  public Command increaseHeight(DoubleSupplier speed) {
    double min = ElevatorHeights.ELEVATOR_MIN_HEIGHT;
    double max = ElevatorHeights.ELEVATOR_MAX_HEIGHT;
    return runOnce(
        () -> {
          double height = goal.position / ElevatorHeights.ELEVATOR_GEAR_RATIO + speed.getAsDouble();
          goal.position =
              Math.max(min, Math.min(height, max)) * ElevatorHeights.ELEVATOR_GEAR_RATIO;
        });
  }

  public void resetEncoder() {
    leaderMotor.setPosition(0);
    followerMotor.setPosition(0);
  }

  public Matter getMatter() {
    double elvMin = ElevatorHeights.ELEVATOR_MIN_HEIGHT;
    double elvMax = ElevatorHeights.ELEVATOR_MAX_HEIGHT;

    double percentHeight = (getHeight() - elvMin) / elvMax;
    double heightCOM = ((elvMax - elvMin) * percentHeight) + elvMin;
    Translation3d centerOfMass =
        new Translation3d(DYNAMICS.COM_LOCATION.getX(), DYNAMICS.COM_LOCATION.getY(), heightCOM);

    double mass = DYNAMICS.TOTAL_MASS;

    return new Matter(centerOfMass, mass);
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
    SmartDashboard.putNumber(
        "elevator height",
        leaderMotor.getPosition().getValueAsDouble() / ElevatorHeights.ELEVATOR_GEAR_RATIO);
    SmartDashboard.putNumber(
        "elevator setpoint", goal.position / ElevatorHeights.ELEVATOR_GEAR_RATIO);
    if (getHeight() < 0.5 && getTargetHeight() < 4.5) {
      leaderMotor.setVoltage(0);
      return;
    }

    if (getHeight() < 4.5 && getTargetHeight() < 4.5) {
      leaderMotor.setVoltage(-1.5);
      return;
    }

    if (goal.position / ElevatorHeights.ELEVATOR_GEAR_RATIO > 78
        && leaderMotor.getPosition().getValueAsDouble() / ElevatorHeights.ELEVATOR_GEAR_RATIO
            > 78) {
      if (leaderMotor.getPosition().getValueAsDouble() / ElevatorHeights.ELEVATOR_GEAR_RATIO > 82) {
        leaderMotor.setVoltage(Elevator.Gains.kg_Top - 0.35);
      } else {
        leaderMotor.setVoltage(Elevator.Gains.kg_Top);
      }
    } else if (Math.abs(leaderMotor.getPosition().getValueAsDouble() - goal.position) < 0.15) {
      if (hasBall.getAsBoolean()) {
        leaderMotor.setVoltage(Elevator.Gains.kg_Ball);
      } else {
        leaderMotor.setVoltage(Elevator.Gains.kG);
      }
    } else {
      leaderMotor.setControl(m_request.withPosition(goal.position).withSlot(slotNum));
    }
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
