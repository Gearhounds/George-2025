// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final XboxController opController = new XboxController(0);
  private final Joystick driverLeft = new Joystick(1);
  private final Joystick driverRight = new Joystick(2);
  public RobotContainer() {
    opController.getLeftY();
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> driverLeft.getRawAxis(1), 
      () -> driverLeft.getRawAxis(0), 
      () -> -driverRight.getRawAxis(0), 
      () -> true));
      SmartDashboard.putBoolean("Running Robot Container", true);
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverRight, 1).onTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));

    new JoystickButton(opController, 1).whileTrue(Commands.run(() -> armSubsystem.setWristSpeed(-opController.getRightY())));
    
    new JoystickButton(opController, 2).whileTrue(Commands.run(() -> armSubsystem.setArmSpeed(-opController.getLeftY())));
    new JoystickButton(opController, 2).whileFalse(Commands.run(() -> armSubsystem.stopArm()));
    
    new Trigger(() -> opController.getLeftBumperButton()||opController.getRightBumperButton()).whileFalse(Commands.run(() -> armSubsystem.armStay()));
    new JoystickButton(opController, 5).whileTrue(Commands.run(() -> armSubsystem.retractArm()));
    new JoystickButton(opController, 6).whileTrue(Commands.run(() -> armSubsystem.extendArm()));
    
    new JoystickButton(opController, 3).whileTrue(Commands.run(() -> armSubsystem.vacOn()));
    new JoystickButton(opController, 3).whileFalse(Commands.run(() -> armSubsystem.vacOff()));
    
    new JoystickButton(opController, 7).whileTrue(Commands.run(() -> armSubsystem.clawClose()));
    new JoystickButton(opController, 8).whileTrue(Commands.run(() -> armSubsystem.clawOpen()));
    new Trigger(() -> opController.getRawButton(7)||opController.getRawButton(8)).whileFalse(Commands.run(() -> armSubsystem.clawOff()));

    new JoystickButton(opController, 9).whileTrue(Commands.run(() -> armSubsystem.climbOn()));
    new JoystickButton(opController, 9).whileFalse(Commands.run(() -> armSubsystem.climbOff()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
