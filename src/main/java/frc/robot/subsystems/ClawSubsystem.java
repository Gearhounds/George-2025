package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MathHelp;

public class ClawSubsystem extends SubsystemBase {
    SparkMax wristMotor = new SparkMax(Constants.ArmConstants.kWristMotorID, MotorType.kBrushed);
    SparkMaxConfig wristConfig = new SparkMaxConfig();

    RelativeEncoder wristEncoder = wristMotor.getEncoder();

    public final PIDController wristPidController = new PIDController(Constants.ArmConstants.WristkP,
                                                                          Constants.ArmConstants.WristkI,
                                                                          Constants.ArmConstants.WristkD);

    public double desiredPosition;
    public ClawSubsystem () {
        desiredPosition = 0;
        wristConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
            wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        wristPidController.setTolerance(0.01);
    }

    public void setWristSpeed(XboxController controller) {
        wristMotor.set(controller.getRightY()); 
    }

    public double getWristPosition() {
        double currentPos = -wristMotor.getEncoder().getPosition();
        currentPos = MathHelp.map(currentPos, 0, .600000023, 0, 1);
        // currentPos = currentPos < 0 ? 0 : currentPos;
        return currentPos;
    }

    public void runToPos() {
        double pidOutput = wristPidController.calculate(getWristPosition(), desiredPosition);
        pidOutput = pidOutput < 0 ? pidOutput/4 : pidOutput;
        wristMotor.set(-pidOutput);
        SmartDashboard.putNumber("claw desired position", desiredPosition);
        SmartDashboard.putNumber("claw current position", getWristPosition());
        SmartDashboard.putNumber("claw pid output", pidOutput);
    }
}
