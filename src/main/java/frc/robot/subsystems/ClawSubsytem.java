package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsytem extends SubsystemBase {
    SparkMax wristMotor = new SparkMax(Constants.ArmConstants.kWristMotorID, MotorType.kBrushed);
    SparkMaxConfig wristConfig = new SparkMaxConfig();

    RelativeEncoder wristEncoder = wristMotor.getEncoder();

    public final PIDController wristPidController = new PIDController(Constants.ArmConstants.WristkP,
                                                                          Constants.ArmConstants.WristkI,
                                                                          Constants.ArmConstants.WristkD);

    double desiredPosition;
    public ClawSubsytem () {
        desiredPosition = 0;
        wristConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
            wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        wristPidController.setTolerance(0.01);
    }

    public double getWristPosition() {
        double currentPos = wristMotor.getEncoder().getPosition();
        currentPos = MathHelp.map(currentPos, 0, .600000023, 0, 1);
        currentPos = currentPos < 0 ? 0 : currentPos;
        return currentPos;
    }

    public void runToPos() {
        double pidOutput = armLengthPidController.calculate(getWristPosition(), desiredPosition);
        extenderMotor.set(pidOutput);
        SmartDashboard.putNumber("desired position", desiredPosition);
        SmartDashboard.putNumber("current position", getArmExtension());
        SmartDashboard.putNumber("pid output", pidOutput);
    }
}
