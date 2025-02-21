package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmExtensionCmd extends Command {
    
    private final ArmSubsystem armSystem;
    private final Supplier<Boolean> shouldRetract, shouldExtend;

    
    private boolean isDone = false;
    
    public ArmExtensionCmd (ArmSubsystem armSubsystem, Supplier<Boolean> retract, Supplier<Boolean> extend) {

        armSystem = armSubsystem;
        shouldRetract = retract;
        shouldExtend = extend;
        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        if (shouldRetract.get() && armSystem.extensionPercent != 0) {
            System.out.println("Retracting");
            armSystem.extensionPercent -= .1;
        }
        if (shouldExtend.get() && armSystem.extensionPercent != 1) {
            System.out.println("Extending");
            armSystem.extensionPercent += .1;
        }
    }

    @Override
    public void execute() {
        isDone = armSystem.armExtension(armSystem.extensionPercent);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        System.out.println("CMD Done");
        return isDone;
    }

}
