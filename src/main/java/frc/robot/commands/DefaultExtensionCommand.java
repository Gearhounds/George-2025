package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtensionSubsystem;

public class DefaultExtensionCommand extends Command{
    private final ExtensionSubsystem extensionSubsystem;

    public DefaultExtensionCommand(ExtensionSubsystem subsystem) {
        this.extensionSubsystem = subsystem;
        addRequirements(extensionSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization code here
    }

    @Override
    public void execute() {
        // Execution code here
        extensionSubsystem.runExtension();
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
