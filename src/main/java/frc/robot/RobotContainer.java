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
import frc.robot.commands.ArmRotationCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ToggleClawCmd;
import frc.robot.commands.WristRotationCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();

  private final XboxController opController = new XboxController(0);
  private final Joystick driverLeft = new Joystick(1);
  private final Joystick driverRight = new Joystick(2);

  private final JoystickButton opButtonA = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_A);
  private final JoystickButton opButtonB = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_B);
  private final JoystickButton opButtonX = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_X);
  private final JoystickButton opButtonY = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_Y);
  private final JoystickButton opRightLittle = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_RIGHT_LITTLE);
  private final JoystickButton opLeftLittle = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_LEFT_LITTLE);
  private final JoystickButton opRightBumper = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_RIGHT_BUMPBER);
  private final JoystickButton opLeftBumper = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_LEFT_BUMPER);
  private final JoystickButton opRightStickDown = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_RIGHTSTICK_DOWN);
  private final JoystickButton opLeftStickDown = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_LEFTSTICK_DOWN);
  private final Trigger opDPadUp = new Trigger(() -> opController.getPOV() == Constants.ControlConstants.OP_STICK_DPAD_UP);
  private final Trigger opDPadDown = new Trigger(() -> opController.getPOV() == Constants.ControlConstants.OP_STICK_DPAD_DOWN);
  private final JoystickButton driverRightRed = new JoystickButton(driverRight, 3);
  private final JoystickButton driverLeftRed = new JoystickButton(driverLeft, 3);

  private boolean isAutoControl = false;

  public RobotContainer() {
    opController.getLeftY();
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> driverLeft.getRawAxis(1), 
      () -> driverLeft.getRawAxis(0), 
      () -> -driverRight.getRawAxis(0), 
      () -> true));
    // armSubsystem.setDefaultCommand(new ArmExtensionCmd(armSubsystem, () -> 0.0));
      SmartDashboard.putBoolean("Running Robot Container", true);
    configureBindings();
  }

  private void configureBindings() {
    // Toggle Auto Control
    opButtonA.onTrue(Commands.run(() -> isAutoControl = !isAutoControl));

    // Zero robot yaw
    new JoystickButton(driverRight, 1).onTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));
    
    // Always run arm pid
    new Trigger(() -> true).whileTrue(Commands.run(() -> armSubsystem.setArmPos()));

    // opButtonB.whileTrue(Commands.run(() -> clawSubsystem.setWristSpeed(opController.getRightY())));

    // opButtonX.whileTrue(Commands.run(() -> armSubsystem.setArmSpeed(-opController.getLeftY())));
    // opButtonX.whileFalse(Commands.run(() -> armSubsystem.stopArm()));
    

    // opLeftBumper.whileTrue(Commands.run(() -> armSubsystem.retractArm()));
    // opRightBumper.whileTrue(Commands.run(() -> armSubsystem.extendArm()));

    // Arm Rotation Setpoints
    opLeftBumper.onTrue(new ArmRotationCmd(armSubsystem, () -> 0.0));
    opRightBumper.onTrue(new ArmRotationCmd(armSubsystem, () -> 0.75));

    // Wrist Rotation Setpoints
    opLeftLittle.onTrue(new WristRotationCmd(clawSubsystem, () -> 0.1));
    opRightLittle.onTrue(new WristRotationCmd(clawSubsystem, () -> 0.9));

    // Arm Extension Setpoints
    opRightStickDown.onTrue(new ArmExtensionCmd(armSubsystem, () -> 0.9));
    opLeftStickDown.onTrue(new ArmExtensionCmd(armSubsystem, () -> 0.1));

    // Open and Close Claw
    driverLeftRed.whileTrue(Commands.run(() -> armSubsystem.clawOpen()));
    driverLeftRed.whileFalse(Commands.run(() -> armSubsystem.clawClose()));

    // Run vacuum
    opButtonY.whileTrue(Commands.run(() -> armSubsystem.vacOn()));
    opButtonY.whileFalse(Commands.run(() -> armSubsystem.vacOff()));

    // Open and Close Climb
    opRightStickDown.whileTrue(new ClimbCmd(armSubsystem, true));
    opRightStickDown.whileFalse(new ClimbCmd(armSubsystem, false));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
