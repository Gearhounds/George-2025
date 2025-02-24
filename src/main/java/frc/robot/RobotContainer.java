// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
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
  private final JoystickButton opRightLittle = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_RIGHT_LITTLE);
  private final JoystickButton opLeftLittle = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_LEFT_LITTLE);
  private final JoystickButton opRightBumper = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_RIGHT_BUMPBER);
  private final JoystickButton opLeftBumper = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_LEFT_BUMPER);
  private final JoystickButton opRightStickDown = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_RIGHTSTICK_DOWN);
  private final JoystickButton opLeftStickDown = new JoystickButton(opController, Constants.ControlConstants.OP_STICK_LEFTSTICK_DOWN);
  private final Trigger opDPadUp = new Trigger(() -> opController.getPOV() == Constants.ControlConstants.OP_STICK_DPAD_UP);
  private final Trigger opDPadDown = new Trigger(() -> opController.getPOV() == Constants.ControlConstants.OP_STICK_DPAD_DOWN);

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
    new JoystickButton(driverRight, 1).onTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));


    opDPadUp.onTrue(new ArmExtensionCmd(armSubsystem, () -> .9));

    opButtonB.whileTrue(Commands.run(() -> armSubsystem.setWristSpeed(opController.getRightY())));

    opButtonX.whileTrue(Commands.run(() -> armSubsystem.setArmSpeed(-opController.getLeftY())));
    opButtonX.whileFalse(Commands.run(() -> armSubsystem.stopArm()));
    
    // new Trigger(() -> opController.getLeftBumperButton() || opController.getRightBumperButton()).onTrue(new ArmExtensionCmd(armSubsystem, () -> opController.getLeftBumperButton(), () -> opController.getRightBumperButton()));
    // new JoystickButton(opController, 5).whileTrue(Commands.run(() -> armSubsystem.retractArm()));
    // new JoystickButton(opController, 6).whileTrue(Commands.run(() -> armSubsystem.extendArm()));

    opLeftBumper.whileTrue(Commands.run(() -> armSubsystem.retractArm()));
    opRightBumper.whileTrue(Commands.run(() -> armSubsystem.extendArm()));

    opButtonY.whileTrue(Commands.run(() -> armSubsystem.vacOn()));
    opButtonY.whileFalse(Commands.run(() -> armSubsystem.vacOff()));
    
    opRightLittle.onTrue(new ToggleClawCmd(armSubsystem));

    opRightStickDown.onTrue(new ArmExtensionCmd(armSubsystem, () -> 0.9));
    opLeftStickDown.onTrue(new ArmExtensionCmd(armSubsystem, () -> 0.1));
    // opRightStickDown.whileTrue(new ClimbCmd(armSubsystem, true));
    // opRightStickDown.whileFalse(new ClimbCmd(armSubsystem, false));
  }

  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(.5, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                () -> new Pose2d(0, 0, new Rotation2d(0)),
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                () -> new Rotation2d(0),
                swerveSubsystem::setModuleStates,
                swerveSubsystem
        );

         // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}
