package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtendToPercentageCmd extends Command {
    
    private final ExtensionSubsystem extensionSystem;
    private double setPosPercent;
    
    public ExtendToPercentageCmd (ExtensionSubsystem extensionSubsystem, Supplier<Double> percent) {
        extensionSystem = extensionSubsystem;
        setPosPercent = percent.get();
        addRequirements(extensionSystem);
    }

    @Override
    public void initialize() {
        if (setPosPercent < 0 || setPosPercent > 1) {
            // dont allow invalid %
            extensionSystem.desiredExtensionPos = extensionSystem.getArmExtension();
        } else {
            extensionSystem.desiredExtensionPos = setPosPercent;
        }
    }

    @Override
    public void execute() {
        extensionSystem.runArmToSetPoint();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return extensionSystem.armLengthPidController.atSetpoint();
    }

}
