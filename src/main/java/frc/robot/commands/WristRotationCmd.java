package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class WristRotationCmd extends Command {
    
    private final ArmSubsystem clawSystem;
    private double setPosPercent;

    
    private boolean isDone = false;
    
    public WristRotationCmd (ClawSubsytem clawSubsystem, Supplier<Double> percent) {

        clawSystem = clawSubsystem;
        setPosPercent = percent.get();
        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        if (setPosPercent < 0 || setPosPercent > 1) {
            // dont allow invalid %
            setPosPercent = clawSystem.getWristPosition();
        } else {
            clawSystem.desiredPosition = setPosPercent;
        }
    }

    @Override
    public void execute() {
        clawSystem.runToPos();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        // System.out.println("CMD Done");
        return armSystem.armLengthPidController.atSetpoint();
    }

}
