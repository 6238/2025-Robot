package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.L1;

@Logged
public class L1Subsystem extends SubsystemBase {
    public TalonFX armMotor;
    public TalonFX intakeMotor;

    public double armTarget = L1.ARM_GROUND;

    public final PositionVoltage armPositionVoltageRequest;

    public L1Subsystem() {
        armMotor = new TalonFX(L1.ARM_MOTOR_ID, "canivore");
        intakeMotor = new TalonFX(L1.INTAKE_MOTOR_ID, "canivore");

        // Configure Arm Motor Constants
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kV = L1.ARM_kV;
        armConfig.Slot0.kA = L1.ARM_kA;
        armConfig.Slot0.kG = L1.ARM_kG;
        armConfig.Slot0.kP = L1.ARM_kV;
        armConfig.Slot0.kI = L1.ARM_kA;
        armConfig.Slot0.kD = L1.ARM_kG;

        armConfig.MotionMagic.MotionMagicCruiseVelocity = L1.ARM_VELOCITY;
        armConfig.MotionMagic.MotionMagicAcceleration = L1.ARM_ACCEL;
        armConfig.MotionMagic.MotionMagicJerk = L1.ARM_JERK;

        // armConfig.Feedback.SensorToMechanismRatio = L1.GEAR_RATIO;

        armMotor.getConfigurator().apply(armConfig);

        armPositionVoltageRequest = new PositionVoltage(0).withSlot(0);
    }

    public double getArmTargetPosition() {
        return armTarget;
    }

    public void setArmPosition(double position) {
        armMotor.setControl(armPositionVoltageRequest.withPosition(position * L1.GEAR_RATIO));
        armTarget = position;
    }

    public Command setArmPositionCommand(DoubleSupplier position) {
        return runOnce(() -> setArmPosition(position.getAsDouble()));
    }

    public void startIntakeWheels() {
        intakeMotor.setVoltage(L1.INTAKE_MOTOR_VOLTAGE);
    }

    public Command startIntakeWheelsCommand() {
        return runOnce(() -> startIntakeWheels());
    }

    public Command holdIntakeWheelsCommand() {
        return runOnce(() -> intakeMotor.setVoltage(2));
    }
    
    public void outtake() {
        intakeMotor.setVoltage(L1.OUTTAKE_MOTOR_VOLTAGE);
    }


    public void stopIntakeWheels() {
        intakeMotor.setVoltage(0.0);
    }

    public Command stopIntakeWheelsCommand() {
        return runOnce(() -> stopIntakeWheels());
    }

    public Command outtakeCommand() {
        return runOnce(() -> outtake());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ARM_MOTOR_VOLTAGE", armMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("ARM_MOTOR_POS", armMotor.getPosition().getValueAsDouble() / L1.GEAR_RATIO);
    }
}
