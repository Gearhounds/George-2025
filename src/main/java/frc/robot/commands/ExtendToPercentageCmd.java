package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtendToPercentageCmd extends Command {
    
    private final ExtensionSubsystem extensionSystem;
    private double setPosPercent;

    
    private boolean isDone = false;
    
    public ExtendToPercentageCmd (ExtensionSubsystem extensionSubsystem, Supplier<Double> percent) {
        extensionSystem = extensionSubsystem;
        setPosPercent = percent.get();
        addRequirements(extensionSystem);
    }

    @Override
    public void initialize() {
        setPosPercent = SmartDashboard.getNumber("Extend Set", 0); // TODO make this generic
        if (setPosPercent < 0 || setPosPercent > 1) {
            // dont allow invalid %
            setPosPercent = extensionSystem.getArmExtension();
        } else {
            extensionSystem.desiredExtensionPos = setPosPercent;
        }
    }

    @Override
    public void execute() {
        extensionSystem.setArmExtensionPos();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        // System.out.println("CMD Done");
        return extensionSystem.armLengthPidController.atSetpoint();
    }

}
