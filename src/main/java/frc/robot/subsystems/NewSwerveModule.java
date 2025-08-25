package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class NewSwerveModule {

    private final String m_name;

    private static final double kModuleMaxAngularVelocity = 2 * Math.PI; // one rotation per second 
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI * 5; // radians per second squared

    private final SparkFlex m_driveMotor;
    private final SparkMax m_turningMotor;

    SparkFlexConfig driveConfig;
    SparkMaxConfig turnConfig;

    private final RelativeEncoder m_driveEncoder;
    private final SparkAbsoluteEncoder m_turningEncoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(.2, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    // private final PIDController m_turningPIDController = new PIDController(.3, ModuleConstants.kITurning, ModuleConstants.kDTurning);
    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
            .8,
            0,
            0,
            new TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);
    
    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorChannel PWM output for the drive motor.
     * @param turningMotorChannel PWM output for the turning motor
     * 
     */

    public NewSwerveModule(
        String name,
        int driveMotorChannel,
        int turningMotorChannel,
        boolean driveMotorReversed,
        boolean turningMotorReversed
    ) {
        m_driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

        driveConfig = new SparkFlexConfig();
        driveConfig
            .inverted(driveMotorReversed)
            .idleMode(IdleMode.kBrake);

        turnConfig = new SparkMaxConfig();
        turnConfig
            .inverted(turningMotorReversed)
            .idleMode(IdleMode.kBrake);
            
        m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_name = name;

    }


    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
    */
    public SwerveModuleState getState() {
        // TODO this seems to be wrong when checking in Advantage Scope
        return new SwerveModuleState(
            m_driveEncoder.getVelocity() * 0.00534, 
            new Rotation2d((m_turningEncoder.getPosition() * Math.PI * 2 - Math.PI))
        );
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() { // used only for odometry 
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition())); 
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public SwerveModuleState setDesiredState(SwerveModuleState desiredState) {
        var encoderRotation = this.getState().angle;

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(encoderRotation);

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        // desiredState.cosineScale(encoderRotation);

        // Calculate the drive output from the drive PID controller.

        // final double driveOutput = // not working right now
        //     m_drivePIDController.calculate(
        //         m_driveEncoder.getVelocity() * (Math.PI * ModuleConstants.kWheelDiameterMeters) / 27 * 4, 
        //         desiredState.speedMetersPerSecond) / DriveConstants.kMaxSpeedMetersPerSecond;

        // final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        // var driveFeedforward = 0;

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            m_turningPIDController.calculate(
                encoderRotation.getRadians(), desiredState.angle.getRadians());

                
        // final double turnFeedforward =
        //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
        var turnFeedforward = 0;
        
        
        SmartDashboard.putNumber(m_name + " desired speed", desiredState.speedMetersPerSecond);
        var forwardSpeedPercent = desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
        SmartDashboard.putNumber(m_name + " forward speed percent", forwardSpeedPercent);
        // m_driveMotor.set(driveOutput + driveFeedforward);
        m_driveMotor.set(forwardSpeedPercent);
        m_turningMotor.set(turnOutput + turnFeedforward);
        return desiredState;
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turningMotor.set(0);
    }

    public void doLogging() {
        var state = getState();

        SmartDashboard.putNumber(m_name + " Drive Encoder Velocity", m_driveEncoder.getVelocity());
        SmartDashboard.putNumber(m_name + " Drive Encoder Position", m_driveEncoder.getPosition());
        
        SmartDashboard.putNumber(m_name + " Turning Encoder Velocity", m_turningEncoder.getVelocity());
        SmartDashboard.putNumber(m_name + " Turning Encoder Position", m_turningEncoder.getPosition());
        
        SmartDashboard.putNumber(m_name + " Current State Speed", state.speedMetersPerSecond);
        SmartDashboard.putNumber(m_name + " Current State Angle", state.angle.getDegrees());
    }


}
