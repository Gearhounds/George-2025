package frc.robot.commands;

import java.io.Console;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
        Supplier<Boolean> fieldOrientedFunction) {
            this.swerveSubsystem = swerveSubsystem;
            this.xSpdFunction = xSpdFunction;
            this.ySpdFunction = ySpdFunction;
            this.turningSpdFunction = turningSpdFunction;
            this.fieldOrientedFunction = fieldOrientedFunction;
            this.xLimiter = new SlewRateLimiter(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
            this.yLimiter = new SlewRateLimiter(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
            this.turningLimiter = new SlewRateLimiter(Constants.ModuleConstants.kTurningEncoderRPM2RadPerSec);
            addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        SmartDashboard.putNumber("X Speed Pre Deadband", xSpeed);
        SmartDashboard.putNumber("Y Speed Pre Deadband", ySpeed);
        SmartDashboard.putNumber("Turn Speed Pre Deadband", turningSpeed);

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > 0.075 ? xSpeed  : 0;
        ySpeed = Math.abs(ySpeed) > 0.075 ? ySpeed  : 0;
        turningSpeed = Math.abs(turningSpeed) > 0.075 ? turningSpeed  : 0;

        SmartDashboard.putNumber("X Speed Post Deadband", xSpeed);
        SmartDashboard.putNumber("Y Speed Post Deadband", ySpeed);
        SmartDashboard.putNumber("Turning Speed Pre Deadband", turningSpeed);

        // 3. Make the driving smoother
        // xSpeed = xLimiter.calculate(xSpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        // ySpeed = yLimiter.calculate(ySpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        // turningSpeed = turningLimiter.calculate(turningSpeed * Constants.DriveConstants.kMaxTurnSpeedRadPerSecond);

        SmartDashboard.putNumber("X Speed Final", xSpeed);
        SmartDashboard.putNumber("Y Speed Final", ySpeed);
        SmartDashboard.putNumber("Turning Speed Final", turningSpeed);
        // 4. Construct desired chassis speeds
        
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        
        swerveSubsystem.setModuleStates(moduleStates);
        SmartDashboard.putBoolean("Running Module State Command", true);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Running End Command", true);
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
