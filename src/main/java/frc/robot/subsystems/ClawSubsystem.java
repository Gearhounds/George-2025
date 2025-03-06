package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MathHelp;
import frc.robot.commands.DefualtClawCmd;
import frc.robot.commands.WristRotationCmd;

public class ClawSubsystem extends SubsystemBase {
    private final SparkMax wristMotor = new SparkMax(Constants.ArmConstants.kWristMotorID, MotorType.kBrushed);
    private SparkMaxConfig wristConfig = new SparkMaxConfig();
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder(); // TODO this is an absolute encoder

    private final XboxController opController;
    
    private final Compressor compressor;
    public final SparkMax vacMotor = new SparkMax(Constants.ArmConstants.kVacMotorID, MotorType.kBrushless);
    
    public final DoubleSolenoid clawSolenoid = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 8, 12);
    

    public final PIDController wristPidController = new PIDController(Constants.ArmConstants.WristkP,
                                                                        Constants.ArmConstants.WristkI,
                                                                        Constants.ArmConstants.WristkD);

    public double desiredRotationPercentage;
    private boolean isManualMode;

    public ClawSubsystem (XboxController opController, Compressor compressor) {
        this.compressor = compressor;
        this.opController = opController;
        isManualMode = true;
        
        
        desiredRotationPercentage = 0;
        wristConfig.inverted(false).idleMode(IdleMode.kBrake);
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        wristPidController.setTolerance(0.01);

        compressor.enableAnalog(90, 100);

        SmartDashboard.putNumber("Claw Set Rotation", 0);
        SmartDashboard.putData("Claw Zero",new WristRotationCmd(this, () -> 0.0));
        SmartDashboard.putData("Claw 100",new WristRotationCmd(this, () -> 1.0));
        SmartDashboard.putData("Claw 50",new WristRotationCmd(this, () -> 0.5));

        setDefaultCommand(new DefualtClawCmd(this));
    }

    @Override
    public void periodic() {
        // Periodic Function for Dashboard and similar
        SmartDashboard.putNumber("Vac Motor: ", vacMotor.get());
        SmartDashboard.putNumber("Claw: Position From Motor", wristMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Claw: SetPoint From PID Controller", wristPidController.getSetpoint());
        SmartDashboard.putNumber("Claw: Set Position From Member", desiredRotationPercentage);
        SmartDashboard.putBoolean("Claw: Is manual", isManualMode);

    }

    // public void initDefaultCommand() {
    //     setDefaultCommand(new DefualtClawCmd(this));
    // }

    public void toggleManualControl() {
        isManualMode = !isManualMode;
    }
    
    // Wrist code

    public double getWristPosition() {
        double currentPos = -wristMotor.getEncoder().getPosition();
        currentPos = MathHelp.map(currentPos, 0, .600000023, 0, 1);
        return currentPos;
    }

    public void runWrist() {
        if (isManualMode) {
            setWristSpeed();
        } else {
            runToPos();
        }
    }

    public void setWristSpeed() {
        wristMotor.set(-opController.getRightY()); 
    }


    public void runToPos() {
        double pidOutput = wristPidController.calculate(getWristPosition(), desiredRotationPercentage);
        pidOutput = pidOutput < 0 ? pidOutput/4 : pidOutput;
        wristMotor.set(-pidOutput);
        SmartDashboard.putNumber("claw desired position", desiredRotationPercentage);
        SmartDashboard.putNumber("claw current position", getWristPosition());
        SmartDashboard.putNumber("claw pid output", pidOutput);
    }

    // end wrist code

    // claw code

    public void clawClose() {
        clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void clawOpen() {
        clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void clawOff() {
        clawSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public Command getOpenClawCommand() {
        return Commands.runOnce(() -> clawOpen());
    }

    public Command getCloseClawCommand() {
        return Commands.runOnce(() -> clawClose());
    }

    public Command getClawOffCommand() {
        return Commands.runOnce(() -> clawOff());
    }

    // end claw code


    // Vac Motor Code 

    public void vacOn() {
        vacMotor.set(1);
    }

    public void vacOff() {
        vacMotor.stopMotor();
    }

    public Command getVacOnCommand() {
        return Commands.runOnce(() -> vacOn());
    }

    public Command getVacOffCommand() {
        return Commands.runOnce(() -> vacOff());
    }

    public Command getToggleVacCommand() {
        return Commands.runOnce(() -> {
            if (vacMotor.get() == 0) {
                vacOn();
            } else {
                vacOff();
            }
        });
    }

    // end Vac Motor Code
}
