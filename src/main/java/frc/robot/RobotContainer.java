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
import frc.robot.commands.ArmExtensionCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ToggleClawCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final XboxController opController = new XboxController(0);
  private final Joystick driverLeft = new Joystick(1);
  private final Joystick driverRight = new Joystick(2);

  private final JoystickButton opButtonA = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_A);
  private final JoystickButton opButtonB = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_B);
  private final JoystickButton opButtonX = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_X);
  private final JoystickButton opButtonY = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_Y);
  private final JoystickButton opButtonRightLittle = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_RIGHT_LITTLE);
  private final JoystickButton opLeftLittle = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_LEFT_LITTLE);
  private final JoystickButton opRightBumper = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_RIGHT_BUMPBER);
  private final JoystickButton opLeftBumper = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_LEFT_BUMPER);
  private final JoystickButton opRightStickDown = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_RIGHTSTICK_DOWN);
  private final JoystickButton opLeftStickDown = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_LEFTSTICK_DOWN);


  public RobotContainer() {
    opController.getLeftY();
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> driverLeft.getRawAxis(1), 
      () -> driverLeft.getRawAxis(0), 
      () -> -driverRight.getRawAxis(0), 
      () -> true));
    armSubsystem.setDefaultCommand(new ArmExtensionCmd(armSubsystem, () -> opController.getRawButton(5), () -> opController.getRawButton(6)));
      SmartDashboard.putBoolean("Running Robot Container", true);
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverRight, 1).onTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));

    // new JoystickButton(opController, Constants.ControlConstants.OP_STICK_B).whileTrue(Commands.run(() -> armSubsystem.setWristSpeed(-opController.getRightY())));
    opButtonB.whileTrue(Commands.run(() -> armSubsystem.setWristSpeed(-opController.getRightY())));

    // new JoystickButton(opController, Constants.ControlConstants.OP_STICK_X).whileTrue(Commands.run(() -> armSubsystem.setArmSpeed(-opController.getLeftY())));
    // new JoystickButton(opController, Constants.ControlConstants.OP_STICK_X).whileFalse(Commands.run(() -> armSubsystem.stopArm()));
    opButtonX.whileTrue(Commands.run(() -> armSubsystem.setArmSpeed(-opController.getLeftY())));
    opButtonX.whileFalse(Commands.run(() -> armSubsystem.stopArm()));
    
    new Trigger(() -> opController.getLeftBumperButton() || opController.getRightBumperButton()).onTrue(new ArmExtensionCmd(armSubsystem, () -> opController.getLeftBumperButton(), () -> opController.getRightBumperButton()));
    // new JoystickButton(opController, 5).whileTrue(Commands.run(() -> armSubsystem.retractArm()));
    // new JoystickButton(opController, 6).whileTrue(Commands.run(() -> armSubsystem.extendArm()));

    opButtonY.whileTrue(Commands.run(() -> armSubsystem.vacOn()));
    opButtonY.whileFalse(Commands.run(() -> armSubsystem.vacOff()));
    
    opButtonRightLittle.onTrue(new ToggleClawCmd(armSubsystem));

    opRightStickDown.whileTrue(new ClimbCmd(armSubsystem, true));
    opRightStickDown.whileFalse(new ClimbCmd(armSubsystem, false));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
