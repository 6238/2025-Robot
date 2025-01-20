package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    // todo
    public boolean isMotorStopped() {
        // if just vel this will flag when you start motor and still accelerating
        double vel = leftMotor.getVelocity().getValueAsDouble();
        double cur = leftMotor.getSupplyCurrent().getValueAsDouble();
        double stall = leftMotor.getMotorStallCurrent().getValueAsDouble(); // todo?
        return Math.abs(vel) <= AlgaeEndEffector.STALL_THRESHOLD && cur > 5; 
    }

    public void setMotorSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(-speed);
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
