package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;



public class ResetArmCmd extends SequentialCommandGroup {
   
    public ResetArmCmd (ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem,
                            ExtensionSubsystem extensionSubsystem) {

        double extensionZero = 0.0;
        double armZero = 0.0;
        double clawZero = 0.25;

        addCommands(new ExtendToPercentageCmd(extensionSubsystem, () -> extensionZero),
                    new ParallelCommandGroup(
                        new ArmRotationCmd(armSubsystem, () -> armZero),
                        new WristRotationCmd(clawSubsystem, () -> clawZero)
                        )
                    );

    }
}