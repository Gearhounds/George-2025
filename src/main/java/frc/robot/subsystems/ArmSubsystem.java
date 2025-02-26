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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MathHelp;

public class ArmSubsystem extends SubsystemBase{

    public final SparkFlex extenderMotor = new SparkFlex(Constants.ArmConstants.kExtenderMotorID, MotorType.kBrushless);

    public final SparkMax vacMotor = new SparkMax(Constants.ArmConstants.kVacMotorID, MotorType.kBrushless); 

    public final SparkFlex rightMotor = new SparkFlex(Constants.ArmConstants.kRightMotorID, MotorType.kBrushless);
    public final SparkFlex leftMotor = new SparkFlex(Constants.ArmConstants.kLeftMotorID, MotorType.kBrushless);
    public final SparkFlexConfig rightConfig = new SparkFlexConfig();
    public final SparkFlexConfig leftConfig = new SparkFlexConfig();

    public final RelativeEncoder armEncoder = rightMotor.getEncoder();
    public final EncoderConfig encoderConfig = new EncoderConfig();

    public final Compressor compressor = new Compressor(21, PneumaticsModuleType.REVPH);

    public final PIDController armAnglePidController = new PIDController(Constants.ArmConstants.AnglekP,
                                                                    Constants.ArmConstants.AnglekI,
                                                                    Constants.ArmConstants.AnglekD);

    public final PIDController armLengthPidController = new PIDController(Constants.ArmConstants.LengthkP,
                                                                          Constants.ArmConstants.LengthkI,
                                                                          Constants.ArmConstants.LengthkD);

    public final Solenoid climbSolenoid = new Solenoid(21, PneumaticsModuleType.REVPH, 10);
    public final DoubleSolenoid clawSolenoid = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 8, 12);

    

    public double desiredPosition;
    public double desiredAngle;
    public ArmSubsystem() {
        desiredPosition = 0;
        desiredAngle = 0;
        compressor.enableAnalog(90, 100);
        
        // extenderMotor.getEncoder().setPosition(0);

        rightConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        armLengthPidController.setTolerance(0.001);
        

        new Thread( () -> {
            try {
                Thread.sleep(1000);
            } catch (Exception e) {
                zeroArmPos();
            }
        });
    }

    

    public void zeroArmPos() {
        armEncoder.setPosition(0);
    }

    public double getArmAngle() {
        double currentArmAngle = armEncoder.getPosition();
        currentArmAngle = MathHelp.map(currentArmAngle, -40, 17, 0, 1);
        // currentArmAngle = currentArmAngle < 0 ? 0 : currentArmAngle;
        return currentArmAngle;
    }

    public double getArmExtension() {
        double currentPos = extenderMotor.getEncoder().getPosition();
        currentPos = MathHelp.map(currentPos, -10, -210, 0, 1);
        currentPos = currentPos < 0 ? 0 : currentPos;
        return currentPos;
    }

    public void stopArm() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public void extendArm() {
        // extenderMotor.set(extenderMotor.getEncoder().getPosition() < -375 ? -.01 : -0.4);
        extenderMotor.set(-1);
    }
    public void retractArm() {
        // extenderMotor.set(extenderMotor.getEncoder().getPosition() > -50 ? .01 : -0.4);
        extenderMotor.set(1);
    }

    
    public void runToPos() {
        double pidOutput = armLengthPidController.calculate(getArmExtension(), desiredPosition);
        extenderMotor.set(-pidOutput);
        SmartDashboard.putNumber("arm desired position", desiredPosition);
        SmartDashboard.putNumber("arm current position", getArmExtension());
        SmartDashboard.putNumber("arm pid output", pidOutput);
    }

    public void armStay() {
        extenderMotor.stopMotor();
    }

    

    public void setArmSpeed(double pos) {
        

        rightMotor.set(pos*.5);
        leftMotor.set(pos*.5);
    }

    public void setArmPos() {
        double pidOutput = armAnglePidController.calculate(getArmAngle(), desiredAngle);
        pidOutput = pidOutput < 0 ? pidOutput/2 : pidOutput;
        rightMotor.set(-pidOutput);
        SmartDashboard.putNumber("claw desired position", desiredAngle);
        SmartDashboard.putNumber("claw current position", getArmAngle());
        SmartDashboard.putNumber("claw pid output", pidOutput);
    }

    public void vacOn() {
        vacMotor.set(1);
    }

    public void vacOff() {
        vacMotor.stopMotor();
    }

    public void clawClose() {
        clawSolenoid.set(Value.kForward);
        // clawSolenoid.set(Value.kOff);
    }

    public void clawOpen() {
        clawSolenoid.set(Value.kReverse);
        // clawSolenoid.set(Value.kOff);
    }

    public void clawOff() {
        clawSolenoid.set(Value.kOff);
    }

    public void climbOn() {
        climbSolenoid.set(true);
    }

    public void climbOff() {
        climbSolenoid.set(false);
    }
}
