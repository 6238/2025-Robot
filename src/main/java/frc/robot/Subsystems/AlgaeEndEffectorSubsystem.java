package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeEndEffector;

public class AlgaeEndEffectorSubsystem extends SubsystemBase {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    public AlgaeEndEffectorSubsystem() {
        leftMotor = new TalonFX(AlgaeEndEffector.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(AlgaeEndEffector.RIGHT_MOTOR_ID);
    }

    public boolean isMotorStopped() {
        double val = leftMotor.getVelocity().getValueAsDouble();
        return Math.abs(val) <= AlgaeEndEffector.STALL_THRESHOLD;
    }

    public void setMotorSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public Command startIntake() {
        return run(() -> setMotorSpeed(AlgaeEndEffector.INTAKE_SPEED));
    }

    public Command intakeUntilStalled() {
        return startIntake().until(() -> isMotorStopped());
    }

    public Command startOutake() {
        return run(() -> setMotorSpeed(AlgaeEndEffector.OUTAKE_SPEED));
    }

    public Command stopMotors() {
        return run(() -> setMotorSpeed(0));
    }

    @Override
    public void periodic() {}
}
