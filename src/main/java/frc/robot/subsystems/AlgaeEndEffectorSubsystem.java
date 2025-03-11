package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AlgaeEndEffector;
import frc.robot.util.OrcestraManager;
import java.util.function.BooleanSupplier;

@Logged
public class AlgaeEndEffectorSubsystem extends SubsystemBase {
  final TalonFX leftMotor;
  final TalonFX rightMotor;

  final PositionVoltage p_request;
  final VelocityVoltage v_request;

  boolean upToSpeed = false;
  boolean velocityControl = true;
  double speedSetpoint = 0.0;

  public AlgaeEndEffectorSubsystem() {
    leftMotor = new TalonFX(AlgaeEndEffector.LEFT_MOTOR_ID);
    rightMotor = new TalonFX(AlgaeEndEffector.RIGHT_MOTOR_ID);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kP = 8.5; // An error of 1 rotation results in 2.4 V output
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

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.SupplyCurrentLimit = 80; // The current limit is always this
    limitConfigs.SupplyCurrentLowerLimit = 20; // But it drops to this...
    limitConfigs.SupplyCurrentLowerTime = 1; // after this amount of time
    limitConfigs.SupplyCurrentLimitEnable = true;

    leftMotor.getConfigurator().apply(limitConfigs, 0.050);
    rightMotor.getConfigurator().apply(limitConfigs, 0.050);

    p_request = new PositionVoltage(0).withSlot(0);
    v_request = new VelocityVoltage(0).withSlot(1);

    OrcestraManager.getInstance().addInstrument(leftMotor);
    OrcestraManager.getInstance().addInstrument(rightMotor);
  }

  /** If either motor's velocity is within percentError of speedSetpoint */
  public boolean upToSpeed(double percentError) {
    double maxError = speedSetpoint * percentError;
    double leftError = Math.abs(leftMotor.getVelocity().getValueAsDouble() + speedSetpoint);
    double rightError = Math.abs(rightMotor.getVelocity().getValueAsDouble() - speedSetpoint);
    return leftError < maxError || rightError < maxError;
  }

  public boolean motorStopped() {
    return upToSpeed
        && (Math.abs(leftMotor.getVelocity().getValueAsDouble() + speedSetpoint)
                > (0.5 * speedSetpoint)
            || Math.abs(rightMotor.getVelocity().getValueAsDouble() - speedSetpoint)
                > (0.5 * speedSetpoint));
  }

  public BooleanSupplier hasBall() {
    return () -> velocityControl == false;
  }

  private void setMotorSpeed(double speed) {
    upToSpeed = upToSpeed(0.1);
    velocityControl = true;
    speedSetpoint = speed;
    leftMotor.setControl(v_request.withVelocity(-speed));
    rightMotor.setControl(v_request.withVelocity(speed));
  }

  private void setMotorPositions(double positionL, double positionR) {
    upToSpeed = false;
    velocityControl = false;
    leftMotor.setControl(p_request.withPosition(positionL));
    rightMotor.setControl(p_request.withPosition(positionR));
  }

  private void disableOutput() {
    speedSetpoint = 0;
    velocityControl = true;
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
    // return run(() ->
    // setMotorSpeed(AlgaeEndEffector.INTAKE_SPEED)).andThen(Commands.waitSeconds(60.0).until(() ->
    // motorStopped()));
    return new SequentialCommandGroup(
        startIntake(), new WaitUntilCommand(() -> motorStopped()), new WaitCommand(0.25));
  }

  public Command holdAlgae() {
    return runOnce(
        () ->
            setMotorPositions(
                leftMotor.getPosition().getValueAsDouble(),
                rightMotor.getPosition().getValueAsDouble()));
  }

  private void setDuty(double speed) {
    velocityControl = false;
    leftMotor.set(-speed);
    rightMotor.set(speed);
  }

  public Command startOutake() {
    return runOnce(() -> setDuty(-0.1));
  }

  public Command startFastOutake() {
    return runOnce(() -> setDuty(-1));
  }

  public Command stopMotors() {
    return runOnce(() -> disableOutput());
  }

  @Override
  public void periodic() {

    if (velocityControl && upToSpeed(0.2)) {
      upToSpeed = true;
    } else {
      upToSpeed = false; // is this breaking?
    }
  }
}
