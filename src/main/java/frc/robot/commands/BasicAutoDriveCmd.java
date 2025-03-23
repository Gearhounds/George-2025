package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class BasicAutoDriveCmd extends Command {
    private final SwerveSubsystem driveSubsystem;

    private double duration;
    private double startTime;

    private ChassisSpeeds speeds;
    SwerveModuleState[] moduleStates;

    public BasicAutoDriveCmd(SwerveSubsystem subsystem, double forwardSpeed, double durationInSeconds) {
        this.driveSubsystem = subsystem;
        this.duration = durationInSeconds * 1000; // convert to ms
        this.speeds = new ChassisSpeeds(forwardSpeed, 0, 0);
        this.moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // Initialization code here
        startTime = System.currentTimeMillis();
        System.out.println("Starting Drive Command at " + startTime);
    }

    @Override
    public void execute() {
        // Execution code here
        System.out.println("Driving");
        driveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        // Code to run when the command ends
        this.speeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        driveSubsystem.setModuleStates(moduleStates);
        System.out.println("Ending Drive Command at " + System.currentTimeMillis());
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should end
        return System.currentTimeMillis() - startTime >= duration;
    }
}

