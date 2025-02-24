package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsytem extends SubsystemBase {
    SparkMax wristMotor = new SparkMax(Constants.ArmConstants.kWristMotorID, MotorType.kBrushed);
    SparkMaxConfig wristConfig = new SparkMaxConfig();

    RelativeEncoder wristEncoder = wristMotor.getEncoder();


    public ClawSubsytem () {
        
        
    }
}
