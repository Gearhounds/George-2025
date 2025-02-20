package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;

public class ArmSubsystem {

    public final SparkFlex extenderMotor = new SparkFlex(Constants.ArmConstants.kExtenderMotorID, MotorType.kBrushless);

    public final SparkMax vacMotor = new SparkMax(Constants.ArmConstants.kVacMotorID, MotorType.kBrushless); 

    public final SparkFlex rightMotor = new SparkFlex(Constants.ArmConstants.kRightMotorID, MotorType.kBrushless);
    public final SparkFlex leftMotor = new SparkFlex(Constants.ArmConstants.kLeftMotorID, MotorType.kBrushless);
    public final SparkFlexConfig rightConfig = new SparkFlexConfig();
    public final SparkFlexConfig leftConfig = new SparkFlexConfig();

    public final RelativeEncoder armEncoder = rightMotor.getEncoder();
    public final EncoderConfig encoderConfig = new EncoderConfig();

    public final Compressor compressor = new Compressor(21, PneumaticsModuleType.REVPH);

    public final PIDController armPidController = new PIDController(Constants.ArmConstants.kP,
                                                                    Constants.ArmConstants.kI,
                                                                    Constants.ArmConstants.kD);

    public final Solenoid climbSolenoid = new Solenoid(21, PneumaticsModuleType.REVPH, 10);
    public final DoubleSolenoid clawSolenoid = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 8, 12);

    public final SparkMax wristMotor = new SparkMax(Constants.ArmConstants.kWristMotorID, MotorType.kBrushed);
    public ArmSubsystem() {
        
        compressor.enableAnalog(90, 100);
        

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

    public double getArmPos() {
        return (armEncoder.getPosition() * Constants.ArmConstants.kArmGearRatio) * 360;
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

    public void armStay() {
        extenderMotor.set(0);
    }

    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
    }

    public void setArmSpeed(double pos) {
        // SmartDashboard.putNumber("Arm Position", getArmPos());
        // pos = (pos * 45) + 45;

        
        
        // SmartDashboard.putNumber("SetPoint", pos);
        // double motorSpeed = armPidController.calculate(getArmPos(), pos);
        // SmartDashboard.putNumber("SetSpeed", motorSpeed);
        // rightMotor.set(motorSpeed);
        // leftMotor.set(motorSpeed);

        rightMotor.set(pos*.5);
        leftMotor.set(pos*.5);
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
