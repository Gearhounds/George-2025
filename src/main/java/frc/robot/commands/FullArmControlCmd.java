package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;



public class FullArmControlCmd extends ParallelCommandGroup {
   
    public FullArmControlCmd (ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem,
                              ExtensionSubsystem extensionSubsystem, 
                              Supplier<Double> armRotation,
                              Supplier<Double> extension,
                              Supplier<Double> clawRotation) {

        addCommands(new ArmRotationCmd(armSubsystem, armRotation),
                    new WristRotationCmd(clawSubsystem, clawRotation),
                    new ExtendToPercentageCmd(extensionSubsystem, extension));

    }
}
