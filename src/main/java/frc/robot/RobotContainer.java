// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmExtensionCmd;
import frc.robot.commands.ArmRotationCmd;
import frc.robot.commands.BasicAutoDriveCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ToggleClawCmd;
import frc.robot.commands.WristRotationCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final XboxController opController = new XboxController(0);
  private final Joystick driverLeft = new Joystick(1);
  private final Joystick driverRight = new Joystick(2);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem(opController);
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();

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

  private final Trigger shouldExtend = new Trigger(() -> {
    return (opController.getRightTriggerAxis() > 0.1 && opController.getLeftTriggerAxis() < .1);
  });  
  private final Trigger shouldRetract = new Trigger(() -> {
    return (opController.getLeftTriggerAxis() > 0.1 && opController.getRightTriggerAxis() < .1);
  });  


  public DigitalInput clawSen = new DigitalInput(6);
  private final Trigger clawSensor = new Trigger(() -> clawSen.get());
  public DigitalInput armSen = new DigitalInput(9);
  private final Trigger armSensor = new Trigger(() -> !armSen.get());

  private final Trigger extensionStopped = new Trigger(() -> !(opLeftBumper.getAsBoolean() || opRightBumper.getAsBoolean()));

  private boolean isAutoControl = false;

  public RobotContainer() {
    opController.getLeftY();
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> driverLeft.getRawAxis(1), 
      () -> driverLeft.getRawAxis(0), 
      () -> -driverRight.getRawAxis(0), 
      () -> true));

  
    configureBindings();
  }

  private void configureBindings() {
    // Zero robot yaw
    // driverRightRed.onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading()));
    driverRightRed.onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading()));

    // Toggle between manual and auto control
    opButtonA.onTrue(Commands.runOnce(() -> isAutoControl = !isAutoControl));
    
    // Always run arm pid
    new Trigger(() -> true && isAutoControl).whileTrue(Commands.run(() -> armSubsystem.setArmAnglePos()));
      
    
    
    // Bindings for Auto Control

    // // Arm Rotation Setpoints
    opLeftBumper.and(() -> isAutoControl).onTrue(new ArmRotationCmd(armSubsystem, () -> 0.0));
    opRightBumper.and(() -> isAutoControl).onTrue(new ArmRotationCmd(armSubsystem, () -> 0.75));

    // // Wrist Rotation Setpoints
    opLeftLittle.and(() -> isAutoControl).onTrue(new WristRotationCmd(clawSubsystem, () -> 0.1));
    opRightLittle.and(() -> isAutoControl).onTrue(new WristRotationCmd(clawSubsystem, () -> 0.9));

    // // Arm Extension Setpoints
    opRightStickDown.and(() -> isAutoControl).onTrue(new ArmExtensionCmd(armSubsystem, () -> 0.9));
    opLeftStickDown.and(() -> isAutoControl).onTrue(new ArmExtensionCmd(armSubsystem, () -> 0.1));

    // End Auto Controls


    // Manual Controls

    // Manual Arm Rotation Control
    new Trigger(() -> true).and(() -> !isAutoControl).whileTrue(Commands.run(() -> armSubsystem.setArmAngleSpeed(opController)));
    // new Trigger(() -> true).whileTrue(Commands.runOnce(() -> armSubsystem.setArmAngleSpeed(opController)));

    // Manual Wrist Rotation Control
    new Trigger(() -> true).and(() -> !isAutoControl).whileTrue(Commands.run(() -> clawSubsystem.setWristSpeed(opController)));
    // new Trigger(() -> true).whileTrue(Commands.run(() -> clawSubsystem.setWristSpeed(opController)));

    // Manual Extension Control
    opLeftBumper.and(() -> !isAutoControl).whileTrue(Commands.runOnce(() -> armSubsystem.retractArm()));
    opRightBumper.and(() -> !isAutoControl).whileTrue(Commands.runOnce(() -> armSubsystem.extendArm()));
    
    // opLeftBumper.and(() -> !isAutoControl).whileTrue(Commands.runOnce(() -> armSubsystem.retractArm()));
    // opRightBumper.and(() -> !isAutoControl).whileTrue(Commands.runOnce(() -> armSubsystem.extendArm()));
    // extensionStopped.and(() -> !isAutoControl).whileTrue(Commands.runOnce(() -> armSubsystem.armExtensionStop()));
    // opLeftBumper.whileTrue(Commands.runOnce(() -> armSubsystem.retractArm()));
    // opRightBumper.whileTrue(Commands.runOnce(() -> armSubsystem.extendArm()));
    // extensionStopped.onTrue(Commands.runOnce(() -> armSubsystem.armExtensionStop()));

    // shouldExtend.and((shouldRetract.negate())).whileTrue(Commands.runOnce(() -> armSubsystem.extendArm()));
    // shouldRetract.and((shouldExtend).negate()).whileTrue(Commands.runOnce(() -> armSubsystem.retractArm()));
    // (shouldExtend.and(shouldRetract)).negate().whileTrue(Commands.runOnce(() -> armSubsystem.armExtensionStop()));

    // End Manual Controls




    // Start Controls

    // Stop Arm
    armSensor.onTrue(Commands.runOnce(() -> armSubsystem.stopArm()));
    
    
    // Open and Close Claw
    driverRightRed.whileTrue(Commands.runOnce(() -> armSubsystem.clawOpen()));
    clawSensor.onFalse(Commands.runOnce(() -> armSubsystem.clawClose()));
    // driverLeftRed.onFalse(Commands.runOnce(() -> armSubsystem.clawClose()));
    // driverRightRed.onTrue(Commands.runOnce(() -> armSubsystem.clawOpen()));
    // driverLeftRed.whileFalse(Commands.runOnce(() -> armSubsystem.clawOff()));

    // Run vacuum
    opButtonY.whileTrue(Commands.run(() -> armSubsystem.vacOn()));
    opButtonY.whileFalse(Commands.run(() -> armSubsystem.vacOff()));

    // Open and Close Climb
    opRightStickDown.whileTrue(new ClimbCmd(armSubsystem, true));
    opRightStickDown.whileFalse(new ClimbCmd(armSubsystem, false));
  }

  public Command getAutonomousCommand() {
    return new BasicAutoDriveCmd(swerveSubsystem, 0.5, 2);
  }
}
