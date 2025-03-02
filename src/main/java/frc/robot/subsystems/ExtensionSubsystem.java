package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MathHelp;
import frc.robot.commands.DefaultExtensionCommand;

public class ExtensionSubsystem extends SubsystemBase{
    public final SparkFlex extenderMotor = new SparkFlex(Constants.ArmConstants.kExtenderMotorID, MotorType.kBrushless);

    public final PIDController armLengthPidController = new PIDController(Constants.ArmConstants.LengthkP,
                                                                          Constants.ArmConstants.LengthkI,
                                                                          Constants.ArmConstants.LengthkD);

    private final XboxController opController;

    private final double EXTENSION_SPEED = 0.5;
    public double desiredExtensionPos;
    private double extensionPIDOutput;
    
    private double retractAxis;
    private double extendAxis;

    private boolean isManualMode;

    public ExtensionSubsystem(XboxController opController) {
        this.opController = opController;
        armLengthPidController.setTolerance(0.001);
        retractAxis = 0;
        extendAxis = 0;
        desiredExtensionPos = 0;
        extensionPIDOutput = 0;
        isManualMode = true;

        // setDefaultCommand(Commands.run(() -> this.runExtension()));
        setDefaultCommand(new DefaultExtensionCommand(this));
    }

    @Override
    public void periodic() {
        retractAxis = opController.getLeftTriggerAxis();
        extendAxis = opController.getRightTriggerAxis();

        SmartDashboard.putNumber("Extension: Position From Function", getArmExtension());
        SmartDashboard.putNumber("Extension: Position From Motor", extenderMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Extension: SetPoint From Member", desiredExtensionPos);
        SmartDashboard.putNumber("Extension: SetPoint From PID Controller", armLengthPidController.getSetpoint());
        SmartDashboard.putNumber("Extension: PID Output", extensionPIDOutput);
        SmartDashboard.putBoolean("Extension: Is manual", isManualMode);

        SmartDashboard.putNumber("retract axi", retractAxis);
        SmartDashboard.putNumber("extend axi", extendAxis);
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
            if (extendAxis > 0.1 && retractAxis < .1) {
                manualArmExtension(extendAxis);
            } else if (retractAxis > 0.1 && extendAxis < .1) {
                manualArmExtension(-retractAxis);
            } else {
                stopExtension();
            }
        } else {
            setArmExtensionPos();
        }
    }

    public void setArmExtensionPos() {
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

    
}
