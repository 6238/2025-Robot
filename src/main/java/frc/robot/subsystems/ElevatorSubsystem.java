package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.Elevator.*;

import frc.robot.Constants.Elevator.ElevatorHeights;
import frc.robot.Constants.Elevator.Gains;
import frc.robot.Constants.IDs;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX elevatorMotor;

    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    public ElevatorSubsystem() {
        elevatorMotor = new TalonFX(IDs.ELEVATOR_MOTOR);

        Slot0Configs motorConfig = new Slot0Configs();
        motorConfig.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.kS = Gains.kS;
        motorConfig.kG = Gains.kG;
        motorConfig.kV = Gains.kV;
        motorConfig.kA = Gains.kA;
        motorConfig.kP = Gains.kP;
        motorConfig.kI = Gains.kI;
        motorConfig.kD = Gains.kD;

        elevatorMotor.getConfigurator().apply(motorConfig);
    }
    public Command setHeightCommand(double givenHeight) {
        return run(() -> setHeight(givenHeight));
    }
    
    //// sets the height to a clamped value
    public void setHeight(Double height) {
        double min = ElevatorHeights.ELEVATOR_MIN_HEIGHT;
        double max = ElevatorHeights.ELEVATOR_MAX_HEIGHT;
        goal.position = Math.max(min, Math.min(height, max)) * ElevatorHeights.ELEVATOR_GEAR_RATIO;
    }

    @Override
    public void periodic() {
        setpoint = ELEVATOR_MOTION_PROFILE.calculate(0.020, setpoint, goal);

        positionVoltage.Position = setpoint.position;
        positionVoltage.Velocity = setpoint.velocity;

        elevatorMotor.setControl(positionVoltage);
    }
    public boolean reachedState() {
        double error = elevatorMotor.getPosition().getValueAsDouble() - setpoint.position;
        return Math.abs(error) < ElevatorHeights.REACH_STATE_THRES;
    }


    /* SYSID */

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle angle = Radians.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    elevatorMotor.setVoltage(voltage.magnitude());
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    log.motor("elevator-motor")
                        .voltage(
                            appliedVoltage.mut_replace(elevatorMotor.get() * RobotController.getBatteryVoltage(), Volts)
                        )
                        .angularPosition(
                            angle.mut_replace(elevatorMotor.getPosition().getValueAsDouble(), Rotations)
                        )
                        .angularVelocity(
                            velocity.mut_replace(elevatorMotor.getVelocity().getValueAsDouble(), RotationsPerSecond)
                        );
                },
                this));
    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
