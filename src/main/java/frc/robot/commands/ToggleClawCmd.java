// package frc.robot.commands;

// import frc.robot.subsystems.ArmSubsystem;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj2.command.Command;

// public class ToggleClawCmd extends Command {
//     private final ArmSubsystem armSystem;
//     private boolean isOpen;
//     private boolean isDone = false;
    
//     public ToggleClawCmd (ArmSubsystem armSubsystem) {
//         armSystem = armSubsystem;
//         addRequirements(armSystem);
//     }

//     @Override
//     public void initialize() {
//         isOpen = armSystem.clawSolenoid.get() == DoubleSolenoid.Value.kReverse;
//         if (isOpen) {
//             armSystem.clawClose();
//         } else {
//             armSystem.clawOpen();
//         }
//     }

//     @Override
//     public void execute() {
//         // armSystem.clawOff();
//         isDone = true;
//     }

//     @Override
//     public void end(boolean interrupted) {}

//     @Override
//     public boolean isFinished() {
//         return isDone;
//     }




// }