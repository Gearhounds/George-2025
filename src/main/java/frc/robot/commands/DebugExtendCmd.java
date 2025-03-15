package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ExtensionSubsystem;

public class DebugExtendCmd extends Command {
     
    private final ExtensionSubsystem extensionSystem;
    private double setPosPercent;
    
    public DebugExtendCmd (ExtensionSubsystem extensionSubsystem, Supplier<Double> percent) {
        extensionSystem = extensionSubsystem;
        setPosPercent = percent.get();
        addRequirements(extensionSystem);
    }

    @Override
    public void initialize() {
        // setPosPercent = SmartDashboard.getNumber("Extend Set", 0);
        // if (setPosPercent < 0 || setPosPercent > 1) {
        //     // dont allow invalid %
        //     extensionSystem.desiredExtensionPos = extensionSystem.getArmExtension();
        // } else {
        //     extensionSystem.desiredExtensionPos = setPosPercent;
        // }
        extensionSystem.desiredExtensionPos = setPosPercent;
    }

    @Override
    public void execute() {
        extensionSystem.runArmToSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        // CommandScheduler.getInstance().schedule(new DefaultExtensionCommand(extensionSystem));
    }

    @Override
    public boolean isFinished() {
        // System.out.println("CMD Done");
        return extensionSystem.armLengthPidController.atSetpoint();
        // return false;
    }

}
