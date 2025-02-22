package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ClimbCmd extends Command {
    private final ArmSubsystem armSystem;
    private boolean isDone = false;
    private boolean isOpen = false;
    public ClimbCmd (ArmSubsystem armSubsystem, boolean open) {
        armSystem = armSubsystem;
        isOpen = open;
        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        armSystem.climbSolenoid.set(isOpen);
    }

    @Override
    public void execute() {
        isDone = true;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
