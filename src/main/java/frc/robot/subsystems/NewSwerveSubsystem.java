package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class NewSwerveSubsystem extends SubsystemBase {
    // Define your swerve drive components here
    // For example, motors, encoders, etc.
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final Translation2d m_frontLeftLocation = new Translation2d(0.264, 0.264);
    private final Translation2d m_frontRightLocation = new Translation2d(0.264, -0.264);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.264, 0.264);
    private final Translation2d m_backRightLocation = new Translation2d(-0.264, -0.264);

    private final NewSwerveModule m_frontLeft = new NewSwerveModule("FL", DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftDriveEncoderReversed, DriveConstants.kFrontLeftTurningEncoderReversed);
    private final NewSwerveModule m_frontRight = new NewSwerveModule("FR", DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightDriveEncoderReversed, DriveConstants.kFrontRightTurningEncoderReversed);
    private final NewSwerveModule m_backLeft = new NewSwerveModule("BL", DriveConstants.kBackLeftDriveMotorPort, DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftTurningEncoderReversed);
    private final NewSwerveModule m_backRight = new NewSwerveModule("BR", DriveConstants.kBackRightDriveMotorPort, DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightTurningEncoderReversed);

    private AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    StructArrayPublisher<SwerveModuleState> actualStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ActualStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<ChassisSpeeds> speedsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();

    private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });

    public NewSwerveSubsystem() {
        // Initialize your swerve drive components here
        m_gyro.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update your swerve drive state here
        doLogging();
        SwerveModuleState[] states = new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState(),
        };
        actualStatesPublisher.set(states);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
    */
    public void drive(
        double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {

        var chassisSpeeds = ChassisSpeeds.discretize(
            false // TODO set this to fieldCentric boolean
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, 
                    m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds);

        var swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
                
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        swerveModuleStates[0] = m_frontLeft.setDesiredState(swerveModuleStates[0]);
        swerveModuleStates[1] = m_frontRight.setDesiredState(swerveModuleStates[1]);
        swerveModuleStates[2] = m_backLeft.setDesiredState(swerveModuleStates[2]);
        swerveModuleStates[3] = m_backRight.setDesiredState(swerveModuleStates[3]);

        desiredStatesPublisher.set(swerveModuleStates);
        speedsPublisher.set(new ChassisSpeeds[] {chassisSpeeds});
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );
    }

    public void resetHeading() {
        m_gyro.reset();
    }

    public void doLogging() {
        // Swerve System Logging


        // Module Logging
        m_frontLeft.doLogging();
        m_frontRight.doLogging();
        m_backLeft.doLogging();
        m_backRight.doLogging();
    }
    
    public void debugModule(String moduleName, SwerveModuleState state) {
        switch (moduleName) {
            case "FL":
                m_frontLeft.setDesiredState(state);
                // m_frontLeft.stop();
                break;
            case "FR":
                m_frontRight.setDesiredState(state);
                break;
            case "BL":
                m_backLeft.setDesiredState(state);
                break;
            case "BR":
                m_backRight.setDesiredState(state);
                break;
            default:
                System.out.println("Invalid module name: " + moduleName);
        }
    }


}
