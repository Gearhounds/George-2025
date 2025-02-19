package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class ArmConstants {
        public static final int kRightMotorID = 31;
        public static final int kLeftMotorID = 32;
        public static final int kExtenderMotorID = 33;
        public static final int kWristMotorID = 34;
        public static final int kVacMotorID = 41;
        public static final double kArmGearRatio = 1 / 273.28;

        public static final double kP = 0.005;
        public static final double kI = 0;
        public static final double kD = 0;
    }
    
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = (4/27);
        public static final double kTurningMotorGearRatio = (7/150);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.01; //0.05
        public static final double kITurning = 0;
        public static final double kDTurning = 0;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(20.625);//20.625
        public static final double kWheelBase = Units.inchesToMeters(20.625);//20.625

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
     

            

        public static final double kMaxSpeedMetersPerSecond = 2.3;
       // public static final double kMaxTurnSpeedRadPerSecond = 5 * 2 * Math.PI; // TODO We guessed
       public static final double kMaxTurnSpeedRadPerSecond = 2.5; // TODO We guessed

        // FRONT LEFT
        public static final int kFrontLeftDriveMotorPort = 14;
        public static final int kFrontLeftTurningMotorPort = 4;
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final int kFrontLeftAbsEncoderPort = 9;
        public static final double kFrontLeftAbsEncoderOffsetRad = 0;
        public static final boolean kFrontLeftAbsEncoderReversed = false;

        

        // FRONT RIGHT
        public static final int kFrontRightDriveMotorPort = 13;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final int kFrontRightAbsEncoderPort = 12;
        public static final double kFrontRightAbsEncoderOffsetRad = 0;
        public static final boolean kFrontRightAbsEncoderReversed = true;

        // BACK LEFT
        public static final int kBackLeftDriveMotorPort = 12;
        public static final int kBackLeftTurningMotorPort = 2;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final int kBackLeftAbsEncoderPort = 10;
        public static final double kBackLeftAbsEncoderOffsetRad = 0;
        public static final boolean kBackLeftAbsEncoderReversed = false;

        // BACK RIGHT
        public static final int kBackRightDriveMotorPort = 11;
        public static final int kBackRightTurningMotorPort = 1;
        public static final boolean kBackRightDriveEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;
        public static final int kBackRightAbsEncoderPort = 11;
        public static final double kBackRightAbsEncoderOffsetRad = 0;
        public static final boolean kBackRightAbsEncoderReversed = false;
    }

}
