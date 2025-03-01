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

public class ExtensionSubsystem extends SubsystemBase{
    public final SparkFlex extenderMotor = new SparkFlex(Constants.ArmConstants.kExtenderMotorID, MotorType.kBrushless);

    public final PIDController armLengthPidController = new PIDController(Constants.ArmConstants.LengthkP,
                                                                          Constants.ArmConstants.LengthkI,
                                                                          Constants.ArmConstants.LengthkD);

    private final XboxController opController;

    private final double EXTENSION_SPEED = 0.5;
    public double desiredExtensionPos;
    
    // TODO use these instead of directly getting triggers
    private double retractAxis;
    private double extendAxis;

    private boolean isManualMode;


    public ExtensionSubsystem(XboxController opController) {
        this.opController = opController;
        armLengthPidController.setTolerance(0.001);
        retractAxis = 0;
        extendAxis = 0;
        desiredExtensionPos = 0;
        isManualMode = true;
    }

    @Override
    public void periodic() {
        // Periodic Function for Dashboard and similar
        SmartDashboard.putNumber("Extension: Position From Function", getArmExtension());
        SmartDashboard.putNumber("Extension: Position From Motor", extenderMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Extension: Set Position From Member", desiredExtensionPos);

        retractAxis = opController.getLeftTriggerAxis();
        extendAxis = opController.getRightTriggerAxis();
    }

    public void initDefaultCommand() {
        // setDefaultCommand(new ArmExtensionCmd(this, opController));
        setDefaultCommand(Commands.run(() -> this.runExtensionFromTriggers()));
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

    public void runExtensionFromTriggers() {
        if (isManualMode) {
            if (extendAxis > 0.1 && retractAxis < .1) {
                manualArmExtension(opController.getRightTriggerAxis());
            } else if (retractAxis > 0.1 && extendAxis < .1) {
                manualArmExtension(-opController.getLeftTriggerAxis());
            } else {
                stopExtension();
            }
        } else {
            setArmExtensionPos();
        }
    }

    public void setArmExtensionPos() {
        double pidOutput = armLengthPidController.calculate(getArmExtension(), desiredExtensionPos);
        extenderMotor.set(-pidOutput);
        SmartDashboard.putNumber("extension desired position", desiredExtensionPos);
        SmartDashboard.putNumber("extension current position", getArmExtension());
        SmartDashboard.putNumber("extension pid output", pidOutput);
    }
    
    public void stopExtension() {
        extenderMotor.set(0); 
    }

    public void toggleManualControl() {
        isManualMode = !isManualMode;
    }

    // These are basic testing functions
    public void extendArm() {
        extenderMotor.set(EXTENSION_SPEED);
    }
    public void retractArm() {
        extenderMotor.set(EXTENSION_SPEED);
    }

    
}
