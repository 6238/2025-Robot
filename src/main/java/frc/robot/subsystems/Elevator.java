package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
}
