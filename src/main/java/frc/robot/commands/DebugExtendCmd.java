package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtensionSubsystem;

public class DebugExtendCmd extends Command {
    
    private final ExtensionSubsystem extensionSystem;
    private double setPosPercent;
    
    public DebugExtendCmd (ExtensionSubsystem extensionSubsystem) {
        extensionSystem = extensionSubsystem;
        addRequirements(extensionSystem);
    }

    @Override
    public void initialize() {
        setPosPercent = SmartDashboard.getNumber("Extend Set", 0);
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
        // System.out.println("CMD Done");
        return extensionSystem.armLengthPidController.atSetpoint();
    }

}
