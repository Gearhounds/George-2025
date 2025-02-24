package frc.robot.subsystems;

import java.io.Console;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.DriveConstants.kFrontLeftDriveMotorPort, Constants.DriveConstants.kFrontLeftTurningMotorPort, 
        Constants.DriveConstants.kFrontLeftDriveEncoderReversed,
        Constants.DriveConstants.kFrontLeftTurningEncoderReversed, Constants.DriveConstants.kFrontLeftAbsEncoderPort,
        Constants.DriveConstants.kFrontLeftAbsEncoderOffsetRad, Constants.DriveConstants.kFrontLeftAbsEncoderReversed);
    private final SwerveModule frontRight = new SwerveModule(
        Constants.DriveConstants.kFrontRightDriveMotorPort, Constants.DriveConstants.kFrontRightTurningMotorPort, 
        Constants.DriveConstants.kFrontRightDriveEncoderReversed,
        Constants.DriveConstants.kFrontRightTurningEncoderReversed, Constants.DriveConstants.kFrontRightAbsEncoderPort,
        Constants.DriveConstants.kFrontRightAbsEncoderOffsetRad, Constants.DriveConstants.kFrontRightAbsEncoderReversed);
    private final SwerveModule backLeft = new SwerveModule(
        Constants.DriveConstants.kBackLeftDriveMotorPort, Constants.DriveConstants.kBackLeftTurningMotorPort, 
        Constants.DriveConstants.kBackLeftDriveEncoderReversed,
        Constants.DriveConstants.kBackLeftTurningEncoderReversed, Constants.DriveConstants.kBackLeftAbsEncoderPort,
        Constants.DriveConstants.kBackLeftAbsEncoderOffsetRad, Constants.DriveConstants.kBackLeftAbsEncoderReversed);
    private final SwerveModule backRight = new SwerveModule(
        Constants.DriveConstants.kBackRightDriveMotorPort, Constants.DriveConstants.kBackRightTurningMotorPort, 
        Constants.DriveConstants.kBackRightDriveEncoderReversed,
        Constants.DriveConstants.kBackRightTurningEncoderReversed, Constants.DriveConstants.kBackRightAbsEncoderPort,
        Constants.DriveConstants.kBackRightAbsEncoderOffsetRad, Constants.DriveConstants.kBackRightAbsEncoderReversed);
    
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        new Rotation2d(0),
        getSwereModulePositions(),
        new Pose2d()
    );
    
    private static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    
        public SwerveSubsystem() {
            
            new Thread( () -> {
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                    zeroHeading();
                }
            });
        }
    
        public void zeroHeading() {
            gyro.reset();
        }
    
        public static double getHeading() {
            return gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-getHeading());
    }

    public SwerveModulePosition[] getSwereModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDriveDistance(), Rotation2d.fromDegrees(frontLeft.getAbsEncoder())),
            new SwerveModulePosition(frontRight.getDriveDistance(), Rotation2d.fromDegrees(frontRight.getAbsEncoder())),
            new SwerveModulePosition(backLeft.getDriveDistance(), Rotation2d.fromDegrees(backLeft.getAbsEncoder())),
            new SwerveModulePosition(backRight.getDriveDistance(), Rotation2d.fromDegrees(backRight.getAbsEncoder()))
        };
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        odometer.update(Rotation2d.fromDegrees(getHeading()), getSwereModulePositions());
        SmartDashboard.putNumberArray("Wheel Angles", new double[] {
            frontLeft.getWheelAngleDeg(),
            frontRight.getWheelAngleDeg(),
            backLeft.getWheelAngleDeg(),
            backRight.getWheelAngleDeg()
        });

        SmartDashboard.putNumberArray("Wheel Abs Angle", new double[] {
            frontLeft.getAbsEncoder(),
            frontRight.getAbsEncoder(),
            backLeft.getAbsEncoder(),
            backRight.getAbsEncoder()
        });
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getSwereModulePositions(), pose);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SmartDashboard.putBoolean("Running Set Module States", true);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
        SmartDashboard.putString("FL State", desiredStates[0].toString());
        SmartDashboard.putString("FR State", desiredStates[1].toString());
        SmartDashboard.putString("BL State", desiredStates[2].toString());
        SmartDashboard.putString("BR State", desiredStates[3].toString());
    }
}
