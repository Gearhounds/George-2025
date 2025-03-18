package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class DefualtClawCmd extends Command {
    private final ClawSubsystem clawSubsystem;

    public DefualtClawCmd(ClawSubsystem subsystem) {
        this.clawSubsystem = subsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization code here
        clawSubsystem.clawOpen();
        clawSubsystem.clawOff();

        clawSubsystem.vacOff();
    }

    @Override
    public void execute() {
        // Execution code here
        clawSubsystem.runWrist();
    }

    @Override
    public void end(boolean interrupted) {
        // Code to run when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // Change this condition as needed
    }
    
}
