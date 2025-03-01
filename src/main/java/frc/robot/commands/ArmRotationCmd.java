package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotationCmd extends Command { 
    
    private final ArmSubsystem armSystem;
    private double setPosPercent;

    
    private boolean isDone = false;
    
    public ArmRotationCmd (ArmSubsystem armSubsystem, Supplier<Double> percent) {

        armSystem = armSubsystem;
        setPosPercent = percent.get();
        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        if (setPosPercent < 0 || setPosPercent > 1) {
            // dont allow invalid %
            setPosPercent = armSystem.getArmAngle();
        } else {
            armSystem.desiredArmAnglePercentage = setPosPercent;
        }
    }

    @Override
    public void execute() {
        armSystem.runArmPID();
    }

    @Override
    public void end(boolean interrupted) {
        armSystem.stopArm();
    }

    @Override
    public boolean isFinished() {
        // System.out.println("CMD Done");
        return false;
    }

}
