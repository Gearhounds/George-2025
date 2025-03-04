package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.MathHelp;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

public class SwerveModule {

    public final SparkFlex driveMotor;
    public final SparkMax turningMotor;

    SparkFlexConfig driveConfig;
    SparkMaxConfig turnConfig;
    

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    // private final SparkAbsoluteEncoder absEncoder;
    private final SparkAbsoluteEncoder absEncoder;
    private final EncoderConfig absConfig;
    // private final CANCoder absEncoder;
    private final boolean absEncoderReversed;
    private final double absEncoderOffsetRad;

    public SwerveModule (int driveMotorID, int turningMotorId, boolean driveMotorReversed,
        boolean turningMotorReversed, int absEncoderID, double absEncoderOffset, boolean absEncoderReversed) {

            driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
            turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
            
            this.absEncoderOffsetRad = absEncoderOffset;
            this.absEncoderReversed = absEncoderReversed;
            absConfig = new EncoderConfig();
            absEncoder = turningMotor.getAbsoluteEncoder();
            absConfig
                .inverted(true);

            driveConfig = new SparkFlexConfig();
            driveConfig
                .inverted(driveMotorReversed)
                .idleMode(IdleMode.kBrake);

            turnConfig = new SparkMaxConfig();
            turnConfig
                .inverted(turningMotorReversed)
                .idleMode(IdleMode.kBrake);
            
            driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
            turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
            

            driveEncoder = driveMotor.getEncoder();
            turningEncoder = turningMotor.getEncoder();

            turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
            turningPidController.enableContinuousInput(-180, 180);

        }

        public double getDrivePosition() {
            SmartDashboard.putBoolean("Running Get Drive Position", true);
            return driveEncoder.getPosition();
        }

        public double getTurningPosition() {
            SmartDashboard.putBoolean("Running Get Turn Position", true);
            return turningEncoder.getPosition();
        }

        public double getDriveVelocity() {
            SmartDashboard.putBoolean("Running Get Drive Velo", true);
            return driveEncoder.getVelocity();
        }

        public double getTurningVelocity() {
            SmartDashboard.putBoolean("Running Get Turn Velo", true);
            return turningEncoder.getVelocity();
        }

        public double getAbsEncoder() {
            double angle = (absEncoder.getPosition() * 360) - 180;
            return angle * (absEncoderReversed ? -1 : 1);
        }

        public void resetEncoders() {
            driveEncoder.setPosition(0);
            turningEncoder.setPosition(getAbsEncoder());
        }
        
        public SwerveModuleState getState() {
            SmartDashboard.putBoolean("Running Get States", true);
            return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsEncoder()));
        }
        
        public void setDesiredState(SwerveModuleState desiredState) {
            SmartDashboard.putBoolean("Running Set Desired State", true);
            if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
                stop();
                return;
            }

            Rotation2d angle = new Rotation2d(absEncoder.getPosition());
            
            desiredState.optimize(angle);
            driveMotor.set(desiredState.speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond);
            
            turningMotor.set(MathHelp.isEqualApprox(getAbsEncoder(), desiredState.angle.getDegrees(), 1) ? 0 : turningPidController.calculate(getAbsEncoder(), desiredState.angle.getDegrees() * (absEncoderReversed ? -1 : 1))/2);
            
            SmartDashboard.putNumber("Wheel Set Angle", desiredState.angle.getDegrees());
            SmartDashboard.putNumber("Wheel Set Speed", desiredState.speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond);
            SmartDashboard.putNumber("Wheel Turn Speed", turningPidController.calculate(getAbsEncoder(), desiredState.angle.getRadians()));
            SmartDashboard.putString("Desired State", desiredState.toString());
            SmartDashboard.putNumber("Wheel Angle", getAbsEncoder());
        }

        public void stop() {
            driveMotor.set(0);
            turningMotor.set(0);
        }
}
