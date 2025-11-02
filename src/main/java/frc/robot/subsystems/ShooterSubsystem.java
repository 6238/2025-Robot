package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX shooterWheel;
    private TalonFX feederWheel;

    private final MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0);

    public ShooterSubsystem() {
        shooterWheel = new TalonFX(Shooter.SHOOTER_WHEEL_ID);
        feederWheel = new TalonFX(Shooter.FEEDER_WHEEL_ID);
    }

    public void configure() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Slot0.kP = Shooter.kP;
        configuration.Slot0.kI = Shooter.kI;
        configuration.Slot0.kD = Shooter.kD;
        configuration.Slot0.kV = Shooter.kV;
        configuration.MotionMagic.MotionMagicCruiseVelocity = Shooter.MOTION_MAGIC_VELOCITY;
        configuration.MotionMagic.MotionMagicAcceleration = Shooter.MOTION_MAGIC_ACCEL;
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shooterWheel.getConfigurator().apply(configuration);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        feederWheel.getConfigurator().apply(motorOutputConfigs);
    
    }

    public Command spinupCommand() {
        return runOnce(() -> shooterWheel.setControl(velocityVoltage));
    }

    public Command stopShooterWheelCommand() {
        return runOnce(() -> shooterWheel.stopMotor());
    }

    public Command setFeederWheelCommand() {
        return runOnce(() -> feederWheel.setVoltage(Shooter.FEEDER_VOLTAGE));
    }

    public Command stopFeederWheelCommand() {
        return runOnce(() -> feederWheel.stopMotor());
    }
}
