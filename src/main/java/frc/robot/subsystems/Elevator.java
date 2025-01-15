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
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private TalonFX elevatorMotor;

    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    public Elevator() {
        elevatorMotor = new TalonFX(Constants.IDs.ELEVATOR_MOTOR);

        Slot0Configs motorConfig = new Slot0Configs();
        motorConfig.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.kS = Constants.Elevator.kS;
        motorConfig.kG = Constants.Elevator.kG;
        motorConfig.kV = Constants.Elevator.kV;
        motorConfig.kA = Constants.Elevator.kA;
        motorConfig.kP = Constants.Elevator.kP;
        motorConfig.kI = Constants.Elevator.kI;
        motorConfig.kD = Constants.Elevator.kD;

        elevatorMotor.getConfigurator().apply(motorConfig);
    }

    public void setHeight(Double height) {
        if (height > Constants.Elevator.maxHeight) {
            goal.position = Constants.Elevator.maxHeight * Constants.Elevator.gearRatio;
        }
        if (height < Constants.Elevator.minHeight) {
            goal.position = Constants.Elevator.minHeight * Constants.Elevator.gearRatio;
        }
        goal.position = height * Constants.Elevator.gearRatio;
    }

    public void setRotations(Double rotations) {
        goal.position = rotations;
    }

    @Override
    public void periodic() {
        setpoint = Constants.Elevator.motionProfile.calculate(0.020, setpoint, goal);

        positionVoltage.Position = setpoint.position;
        positionVoltage.Velocity = setpoint.velocity;

        elevatorMotor.setControl(positionVoltage);
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
