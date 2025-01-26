package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AlgaeEndEffector;

public class AlgaeEndEffectorSubsystem extends SubsystemBase {
    final TalonFX leftMotor;
    final TalonFX rightMotor;

    final PositionVoltage p_request;
    final VelocityVoltage v_request;

    public AlgaeEndEffectorSubsystem() {
        leftMotor = new TalonFX(AlgaeEndEffector.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(AlgaeEndEffector.RIGHT_MOTOR_ID);

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;

        slot0Configs.kP = 5.0; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0.2; // no output for integrated error
        slot0Configs.kD = 0.2; // A velocity of 1 rps results in 0.1 V output

        var slot1Configs = talonFXConfigs.Slot1;

        slot1Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot1Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kP = 0.2; // An error of 1 rps results in 0.11 V output
        slot1Configs.kI = 0; // no output for integrated error
        slot1Configs.kD = 0; // no output for error derivative

        leftMotor.getConfigurator().apply(talonFXConfigs, 0.050);
        rightMotor.getConfigurator().apply(talonFXConfigs, 0.050);

        p_request = new PositionVoltage(0).withSlot(0);
        v_request = new VelocityVoltage(0).withSlot(1);
    }

    boolean upToSpeed = false;
    boolean velocityControl = false;
    double speedSetpoint = 0.0;

    public boolean motorStopped() {
        // System.out.println("test");
        return upToSpeed && (Math.abs(leftMotor.getVelocity().getValueAsDouble() + speedSetpoint) > (0.2 * speedSetpoint) || Math.abs(rightMotor.getVelocity().getValueAsDouble() - speedSetpoint) > (0.2 * speedSetpoint));
    }

    private void setMotorSpeed(double speed) {
        upToSpeed = false;
        velocityControl = true;
        speedSetpoint = speed;
        leftMotor.setControl(v_request.withVelocity(-speed));
        rightMotor.setControl(v_request.withVelocity(speed));
    }

    private void setMotorPositions(double positionL, double positionR) {
        velocityControl = false;
        leftMotor.setControl(p_request.withPosition(positionL));
        rightMotor.setControl(p_request.withPosition(positionR));
    }

    private void disableOutput(){
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public Command startIntake() {
        return runOnce(() -> setMotorSpeed(AlgaeEndEffector.INTAKE_SPEED));
    }

    // public Command intakeUntilStalled() {
    //     return startIntake().until(() -> isMotorStopped());
    // }

    public Command intakeUntilStalled() {
        // return run(() -> setMotorSpeed(AlgaeEndEffector.INTAKE_SPEED)).andThen(Commands.waitSeconds(60.0).until(() -> motorStopped()));
        return new SequentialCommandGroup(
            startIntake(),
            // stopMotors(),
            new WaitUntilCommand(() -> motorStopped()),
            new WaitCommand(0.25)
        );

    }

    public Command holdAlgae() {
        return runOnce(() -> setMotorPositions(leftMotor.getPosition().getValueAsDouble(), rightMotor.getPosition().getValueAsDouble()));
    }

    public Command startOutake() {
        return runOnce(() -> setMotorSpeed(-AlgaeEndEffector.OUTAKE_SPEED));
    }

    public Command stopMotors() {
        return runOnce(() -> disableOutput());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intakespeed", -leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("intakeUpToSpeed", upToSpeed);
        if (velocityControl && (Math.abs(leftMotor.getVelocity().getValueAsDouble() + speedSetpoint) < (0.1 * speedSetpoint) || Math.abs(rightMotor.getVelocity().getValueAsDouble() - speedSetpoint) < (0.1 * speedSetpoint))){
            upToSpeed = true;
            // System.err.println("up to speed");
        }
    }
}
