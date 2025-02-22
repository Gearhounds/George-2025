package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;

public class ToggleClawCmd extends Command {
    private final ArmSubsystem armSystem;
    private boolean isOpen;
    private boolean isDone = false;
    
    public ToggleClawCmd (ArmSubsystem armSubsystem) {
        armSystem = armSubsystem;
        isOpen = armSystem.clawSolenoid.get() == DoubleSolenoid.Value.kForward;
        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        if (armSystem.clawSolenoid.get() == DoubleSolenoid.Value.kForward) {
            armSystem.clawSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else {
            armSystem.clawSolenoid.set(DoubleSolenoid.Value.kForward);
        }
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