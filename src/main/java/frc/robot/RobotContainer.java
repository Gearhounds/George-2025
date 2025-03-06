// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.cert.Extension;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmRotationCmd;
import frc.robot.commands.BasicAutoDriveCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final XboxController opController = new XboxController(0);
  private final Joystick driverLeft = new Joystick(1);
  private final Joystick driverRight = new Joystick(2);

  private final Compressor compressor = new Compressor(21, PneumaticsModuleType.REVPH);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(driverLeft, driverRight);
  private final ArmSubsystem armSubsystem = new ArmSubsystem(opController);
  public final ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(opController);
  private final ClawSubsystem clawSubsystem = new ClawSubsystem(opController, compressor);

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
  private final JoystickButton driverRightTrigger = new JoystickButton(driverRight, 2);
  private final JoystickButton driverLeftTrigger = new JoystickButton(driverLeft, 2);
  private final JoystickButton driverRightPinky = new JoystickButton(driverRight, 5);

  private final Trigger shouldExtend = new Trigger(() -> {
    return (opController.getRightTriggerAxis() > 0.1 && opController.getLeftTriggerAxis() < .1);
  });  
  private final Trigger shouldRetract = new Trigger(() -> {
    return (opController.getLeftTriggerAxis() > 0.1 && opController.getRightTriggerAxis() < .1);
  });  


  public DigitalInput clawSen = new DigitalInput(6);
  private final Trigger clawSensor = new Trigger(() -> clawSen.get());
  // public DigitalInput armSen = new DigitalInput(9);
  // private final Trigger armSensor = new Trigger(() -> !armSen.get());

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
    driverRightTrigger.onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading()));
    driverLeftTrigger.onTrue(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> driverLeft.getRawAxis(1), 
      () -> driverLeft.getRawAxis(0), 
      () -> -driverRight.getRawAxis(0), 
      () -> false));

    // Toggle between manual and auto control
    opButtonA.onTrue(getToggleManualControlCommand());
    

    // Manual Controls

    // Manual Arm Rotation Control
    // new Trigger(() -> true).and(() -> !isAutoControl).whileTrue(Commands.run(() -> armSubsystem.setArmAngleSpeed(opController)));
    // new Trigger(() -> true).whileTrue(Commands.runOnce(() -> armSubsystem.runArmManual()));

    // Manual Wrist Rotation Control
    // new Trigger(() -> true).and(() -> !isAutoControl).whileTrue(Commands.run(() -> clawSubsystem.setWristSpeed(opController)));
    // new Trigger(() -> true).whileTrue(Commands.run(() -> clawSubsystem.setWristSpeed(opController)));


    // armSensor.onChange(Commands.runOnce(() -> armSubsystem.toggleAtLimit()));

    // extension bindings
    
    
    
    
    
    
    
    // end extension bindings
    
    // claw bindings
    
    driverRightRed.onTrue(clawSubsystem.getOpenClawCommand());
    driverRightPinky.onTrue(clawSubsystem.getCloseClawCommand());
    clawSensor.onFalse(clawSubsystem.getCloseClawCommand());




    // end claw bindings

    // Vacuum Bindings

    opButtonY.onChange(clawSubsystem.getToggleVacCommand());








    // End Manual Controls




    // Start Controls

    // Stop Arm
    

    // Open and Close Climb
    opRightStickDown.whileTrue(new ClimbCmd(armSubsystem, true));
    opRightStickDown.whileFalse(new ClimbCmd(armSubsystem, false));
  }

  public Command getToggleManualControlCommand() {
    return Commands.runOnce(() -> {
      armSubsystem.toggleManualControl();
      extensionSubsystem.toggleManualControl();
      clawSubsystem.toggleManualControl();
    });
  }

  public Command getAutonomousCommand() {
    return new BasicAutoDriveCmd(swerveSubsystem, 0.5, 2);
  }
}
