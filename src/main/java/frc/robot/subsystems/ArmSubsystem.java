package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MathHelp;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.DefaultArmCmd;

public class ArmSubsystem extends SubsystemBase {

    public final SparkFlex rightMotor = new SparkFlex(Constants.ArmConstants.kRightMotorID, MotorType.kBrushless);
    public final SparkFlex leftMotor = new SparkFlex(Constants.ArmConstants.kLeftMotorID, MotorType.kBrushless);
    public final SparkFlexConfig rightConfig = new SparkFlexConfig();
    public final SparkFlexConfig leftConfig = new SparkFlexConfig();

    public final RelativeEncoder armEncoder = rightMotor.getEncoder();
    public final EncoderConfig encoderConfig = new EncoderConfig();

    private double armAnglePIDOutput; // this is the speed of the motor after being run through PID
    public double desiredArmAnglePercentage; // this is the setpoint of the arm PID

    public final PIDController armAnglePidController = new PIDController(Constants.ArmConstants.AnglekP,
                                                                        Constants.ArmConstants.AnglekI,
                                                                        Constants.ArmConstants.AnglekD);

    public final Solenoid climbSolenoid = new Solenoid(21, PneumaticsModuleType.REVPH, 10);
    

    private final XboxController opController;

    private boolean isManualMode;

    private boolean atLimit;

    public DigitalInput armSen = new DigitalInput(9);

    public ArmSubsystem(XboxController opController) {
        isManualMode = true;
        atLimit = false;

        desiredArmAnglePercentage = 0;
        this.opController = opController;

        rightConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .follow(rightMotor, true);

        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        zeroArmPos();

        setDefaultCommand(new DefaultArmCmd(this));
    }

    public void periodic() {
        SmartDashboard.putNumber("arm desired position", desiredArmAnglePercentage);
        SmartDashboard.putNumber("arm current position", getArmAngle());
        SmartDashboard.putNumber("arm pid output", armAnglePIDOutput);
        SmartDashboard.putBoolean("Arm: Is manual", isManualMode);
        SmartDashboard.putBoolean("Arm Sensor", armSen.get());
    }

    public void toggleAtLimit() {
        atLimit = !atLimit;
    }

    public void toggleManualControl() {
        isManualMode = !isManualMode;
    }

    public void zeroArmPos() {
        armEncoder.setPosition(0);
    }

    public double getArmAngle() {
        double currentArmAngle = armEncoder.getPosition();
        currentArmAngle = MathHelp.map(currentArmAngle, -40, 17, 0, 1);
        return currentArmAngle;
    }

    public void stopArm() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public void runArm() {
        System.out.println("Running arm");
        if (isManualMode) {
            runArmManual();
        } else {
            runArmPID();
        }
    }

    public void runArmManual() {
        var speed = opController.getLeftY() * 0.5;
        if (speed < 0 && !armSen.get()) speed = 0;
        rightMotor.set(speed); // left motor is inverted follower
    }

    public void runArmPID() {
        double rawPID = armAnglePidController.calculate(getArmAngle(), desiredArmAnglePercentage);
        armAnglePIDOutput = rawPID < 0 ? rawPID / 2 : rawPID; // limit the speed of the arm when going down
        rightMotor.set(armAnglePIDOutput);
    }

    public void climbOn() {
        climbSolenoid.set(true);
    }

    public void climbOff() {
        climbSolenoid.set(false);
    }

    public Command getClimbStartCommand() {
        return new ClimbCmd(this, true);
    }

    public Command getClimbCloseCmd() {
        return new ClimbCmd(this, false);
    }
}
