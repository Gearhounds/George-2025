package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotationCmd extends Command { 
    
    private final ArmSubsystem armSystem;
    private double setPosPercent;
    private boolean stop = false;
    
    public ArmRotationCmd (ArmSubsystem armSubsystem, Supplier<Double> percent) {
        armSystem = armSubsystem;
        setPosPercent = percent.get();
        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        // setPosPercent = SmartDashboard.getNumber("Arm Angle PID SET", setPosPercent);
        if (setPosPercent < 0 || setPosPercent > 1) {
            // dont allow invalid %
            setPosPercent = armSystem.getArmAngle();
        } else {
            armSystem.desiredArmAnglePercentage = setPosPercent;
        }
        stop = true;
    }

    @Override
    public void execute() {
        armSystem.runArmPID();
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(new DefaultArmCmd(armSystem));
    }

    @Override
    public boolean isFinished() {
        // System.out.println("CMD Done");
        return armSystem.armAnglePidController.atSetpoint();
    }

}
