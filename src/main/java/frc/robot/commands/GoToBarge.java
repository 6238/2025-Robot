package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Swerve.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAX_SPEED;

import frc.robot.Constants.AutoMoveGeneration;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeEndEffector;
import frc.robot.subsystems.SwerveSubsystem;

public class GoToBarge extends Command {
    DoubleSupplier m_xAxisSupplier;
    DoubleSupplier m_yAxisSupplier;
    DoubleSupplier m_angleAxisSupplier;
    SwerveSubsystem m_swerve;

    public GoToBarge(SwerveSubsystem swerve, DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier, DoubleSupplier angleAxisSupplier) {
        m_swerve = swerve;
        m_xAxisSupplier = xAxisSupplier;
        m_yAxisSupplier = yAxisSupplier;
        m_angleAxisSupplier = angleAxisSupplier;
    }

    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        double sign = 1.0;
        if (ally.isPresent()) {
            sign = (ally.get() == Alliance.Blue) ? 1.0 : -1.0;
        }
        // Make the robot move
        double yInput = Math.pow(m_yAxisSupplier.getAsDouble(), 3);
        double rotation = m_angleAxisSupplier.getAsDouble() * MAX_ANGULAR_VELOCITY;

        Translation2d translation = new Translation2d(sign * AutoMoveGeneration.BARGE_SPEED * MAX_SPEED, sign * yInput * MAX_SPEED);
        m_swerve.drive(translation, rotation, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(ChassisSpeeds.discretize(0, 0, 0, 1));
    }

    private boolean tryingToDrive() {
        return Math.abs(m_xAxisSupplier.getAsDouble()) > 0;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        double poseX = AutoMoveGeneration.BARGE_X_BLUE;
        if (ally.isPresent() && ally.get() == Alliance.Red) {
            poseX = AutoMoveGeneration.BARGE_X_RED;
        }
        return tryingToDrive() || Math.abs(poseX - m_swerve.getPose().getX()) < AutoMoveGeneration.BARGE_THRESHOLD;
    }
}
