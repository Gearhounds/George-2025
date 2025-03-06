package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MathHelp;
import frc.robot.commands.DebugExtendCmd;
import frc.robot.commands.DefaultExtensionCommand;
import frc.robot.commands.ExtendToPercentageCmd;

public class ExtensionSubsystem extends SubsystemBase{
    public final SparkFlex extenderMotor = new SparkFlex(Constants.ArmConstants.kExtenderMotorID, MotorType.kBrushless);

    public final PIDController armLengthPidController = new PIDController(Constants.ArmConstants.LengthkP,
                                                                          Constants.ArmConstants.LengthkI,
                                                                          Constants.ArmConstants.LengthkD);

    private final XboxController opController;

    private final double EXTENSION_SPEED = 0.5;
    public double desiredExtensionPos;
    private double extensionPIDOutput;
    
    private double retractSpeed;
    private double extendSpeed;

    private boolean isManualMode;

    public ExtensionSubsystem(XboxController opController) {
        this.opController = opController;
        armLengthPidController.setTolerance(0.01);
        retractSpeed = 0;
        extendSpeed = 0;
        desiredExtensionPos = 0;
        extensionPIDOutput = 0;
        isManualMode = true;

        SmartDashboard.putNumber("Extend Set", 0);

        setDefaultCommand(new DefaultExtensionCommand(this));
    }

    @Override
    public void periodic() {
        retractSpeed = opController.getLeftTriggerAxis();
        extendSpeed = opController.getRightTriggerAxis();

        SmartDashboard.putNumber("Extension: Position From Function", getArmExtension());
        SmartDashboard.putNumber("Extension: Position From Motor", extenderMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Extension: SetPoint From Member", desiredExtensionPos);
        SmartDashboard.putNumber("Extension: SetPoint From PID Controller", armLengthPidController.getSetpoint());
        SmartDashboard.putNumber("Extension: PID Output", extensionPIDOutput);
        SmartDashboard.putBoolean("Extension: Is manual", isManualMode);

        SmartDashboard.putNumber("retract speed", retractSpeed);
        SmartDashboard.putNumber("extend speed", extendSpeed);
    }

    public void toggleManualControl() {
        isManualMode = !isManualMode;
    }

    public double getArmExtension() {
        double currentPos = extenderMotor.getEncoder().getPosition();
        currentPos = MathHelp.map(currentPos, -10, -210, 0, 1);
        currentPos = currentPos < 0 ? 0 : currentPos;
        return currentPos;
    }

    public void manualArmExtension(double speed) {
        extenderMotor.set(speed);
    }

    public void runExtension() {
        if (isManualMode) {
            if (extendSpeed > 0.1 && retractSpeed < .1) {
                manualArmExtension(-extendSpeed);
            } else if (retractSpeed > 0.1 && extendSpeed < .1) {
                manualArmExtension(retractSpeed);
            } else {
                stopExtension();
            }
        } else {
            runArmToSetPoint();
        }
    }

    public void runArmToSetPoint() {
        extensionPIDOutput = armLengthPidController.calculate(getArmExtension(), desiredExtensionPos);
        extenderMotor.set(-extensionPIDOutput);
    }
    
    public void stopExtension() {
        extenderMotor.set(0); 
    }

    // These are basic testing functions
    public void extendArm() {
        extenderMotor.set(EXTENSION_SPEED);
    }
    public void retractArm() {
        extenderMotor.set(EXTENSION_SPEED);
    }

    public Command getExtendToZeroCmd() {
        return new ExtendToPercentageCmd(this, () -> 0.0);
    }

    public Command getExtendToMaxCmd() {
        return new ExtendToPercentageCmd(this, () -> 1.0);
    }

    public Command getExtendToDebugCmd() {
        return new DebugExtendCmd(this);
    }
    

    
}
