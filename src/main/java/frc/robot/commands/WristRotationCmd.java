// package frc.robot.commands;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ClawSubsystem;

// public class WristRotationCmd extends Command { 
    
//     private final ClawSubsystem clawSystem;
//     private double setPosPercent;

    
//     public WristRotationCmd (ClawSubsystem clawSubsystem, Supplier<Double> percent) {

//         clawSystem = clawSubsystem;
//         setPosPercent = percent.get();
//         addRequirements(clawSystem);
//     }

//     @Override
//     public void initialize() {
//         if (setPosPercent < 0 || setPosPercent > 1) {
//             // dont allow invalid %
//             setPosPercent = clawSystem.getWristPosition();
//         } else {
//             clawSystem.desiredRotationPercentage = setPosPercent;
//         }
//     }

//     @Override
//     public void execute() {
//         clawSystem.runToPos();
//     }

//     @Override
//     public void end(boolean interrupted) {}

//     @Override
//     public boolean isFinished() {
//         // System.out.println("CMD Done");
//         return clawSystem.wristPidController.atSetpoint();
//     }

// }
