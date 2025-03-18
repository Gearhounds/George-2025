package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCmd extends Command {
    private final ArmSubsystem armSubsystem;

    public DefaultArmCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization code here
    }

    @Override
    public void execute() {
        // Execution code here
        armSubsystem.runArm();
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
