package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ClawSubsystem;

public class WristRotationCmd extends Command  { 
    
    private final ClawSubsystem clawSystem;
    private double setPosPercent;

    
    public WristRotationCmd (ClawSubsystem clawSubsystem, Supplier<Double> percent) {
        clawSystem = clawSubsystem;
        setPosPercent = percent.get();
        addRequirements(clawSystem);
    }

    @Override
    public void initialize() {
        // setPosPercent = SmartDashboard.getNumber("Claw Set Rotation", 0);
        if (setPosPercent < 0 || setPosPercent > 1) {
            // dont allow invalid %
            setPosPercent = clawSystem.getWristPosition();
        } else {
            clawSystem.desiredRotationPercentage = setPosPercent;
        } 
    }

    @Override
    public void execute() {
        clawSystem.runToPos();
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(new DefualtClawCmd(clawSystem));
    }

    @Override
    public boolean isFinished() {
        // System.out.println("CMD Done");
        return clawSystem.wristPidController.atSetpoint();
    }

}
