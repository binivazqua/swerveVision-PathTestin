/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.alignConstraints;

public final class Constants {

    public static final class ModuleConstants {

        //Diameter of the Wheel in meters
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
       
        //Gear ratio of the drive motor
        public static final double kDriveMotorGearRatio = 1 / 7.13;
        
        //Gear ratio of the turning motor
        public static final double kTurningMotorGearRatio = 1 /  13.71;
        
        // Drive position conversion factor from rotation to meters
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
       
        // Turning position conversion factor from rotation to radias
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
       
        // Drive velocity conversion factor from RPM to M/S
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
       
        // Turning velocity conversion factor from RPM to Rads/Sec
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
       
        //P constant for the turn motor
        public static final double kPTurning = 0.247;
    }

    public static final class DriveConstants {

         /**
         * 
         *             Trackwidth
         *     --------------------------
         *     |                        |
         *     |                        |
         *     |                        | |
         *     |                        | |WHEELBASE
         *     |        midpoint        | | 
         *     |                        |
         *     |                        |
         *     |                        |
         *     |                        |
         *     --------------------------
         */


        

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(19.25);
        
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(19.25);
       
        public static final double DRIVE_BASE_RADIUS = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        
        /**
        * Create the kinematics of the swerve
        */
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kMaxDriveVEL = 6.5;
        public static final double kMaxRotVEL = 3 * 2 * Math.PI;

        public static final double kDriveLimiter = kMaxDriveVEL / 3;
        public static final double kRotationLimiter = //
                kMaxRotVEL / 3;
        public static final double kDriveAccelerationLimiter = 5;
        public static final double kRotationAccelerationLimiter = 7;
    }

    public static class MODS {
  

        public static final class frontLeftModule {
                  
            public static int driveMotorID = 1;
            public static int turningMotorID = 2;
            public static boolean driveMotorInverted = true;
            public static boolean turningMotorInverted = true;
            public static double absoluteEncoderOffsetRad = 0;
            public static boolean absoluteEncoderReversed = false;
                       
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
                  
         }
                  
        public static final class frontRightModule {
                  
            public static int driveMotorID = 3;
            public static int turningMotorID = 4;
            public static boolean driveMotorInverted = true;
            public static boolean turningMotorInverted = true;
            public static double absoluteEncoderOffsetRad = 0;
            public static boolean absoluteEncoderReversed = false;
                      
                  
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
            
                  
        }
                  
        public static final class rearLeftModule {
                  
            public static int driveMotorID = 5;
            public static int turningMotorID = 6;
            public static boolean driveMotorInverted = true;
            public static boolean turningMotorInverted = true;
            public static int absoluteEncoderID = 3;
            public static double absoluteEncoderOffsetRad = 0;
            public static boolean absoluteEncoderReversed = false;
                  
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
            
                  
        }
                  
        public static final class rearRightModule {
                  
            public static int driveMotorID = 7;
            public static int turningMotorID = 8;
            public static boolean driveMotorInverted = true;
            public static boolean turningMotorInverted = true;  
            public static int absoluteEncoderID =  4;
            public static double absoluteEncoderOffsetRad = 0;
            public static boolean absoluteEncoderReversed = false; 
                  
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
            
                  
        }
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxDriveVEL / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kMaxRotVEL / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        
        
       
    }

    public static final class limelightConstants {


    /**
     * PID constants for the autoalign
     */
     

        public static final class aprilTag{

            public static final double kPdrive = 2;
            public static final double kIdrive = 0;
            public static final double kDdrive = 0;

            public static final double kPstrafe = 0.1;
            public static final double kIstrafe = 0;
            public static final double kDstrafe = 0;

            public static final double kProtation = 0.06;
            public static final double kIrotation = 0;
            public static final double kDrotation = 0;

            public static final TrapezoidProfile.Constraints driveConstraints = 
                new TrapezoidProfile.Constraints(9,9);

            
            public static final TrapezoidProfile.Constraints strafeConstraints = 
                new TrapezoidProfile.Constraints(3,2);

            
            public static final TrapezoidProfile.Constraints rotationConstraints = 
                new TrapezoidProfile.Constraints(3,2);
            
            public static double driveOffset = 0.48;
            public static double strafeOffset = 2.3;
            public static double rotationOffset = -21.8;
            
            public static ProfiledPIDController drivePID = new ProfiledPIDController(kPdrive, kIdrive, kDdrive, driveConstraints);
            public static ProfiledPIDController strafePID = new ProfiledPIDController(kPstrafe, kIstrafe, kDstrafe, strafeConstraints);
            public static ProfiledPIDController rotationPID = new ProfiledPIDController(kProtation, kIrotation, kDrotation, rotationConstraints);



            public static final alignConstraints constraints =  
        new alignConstraints(driveOffset, strafeOffset, rotationOffset, drivePID, strafePID, rotationPID);

        }

        public static final class noteOffsets{

            public static final double kPdrive = 0.1;
            public static final double kIdrive = 0;
            public static final double kDdrive = 0;

            public static final double kPstrafe = 0.08;
            public static final double kIstrafe = 0;
            public static final double kDstrafe = 0;

            public static final double kProtation = 0.04;
            public static final double kIrotation = 0;
            public static final double kDrotation = 0;

            public static final TrapezoidProfile.Constraints driveConstraints = 
                new TrapezoidProfile.Constraints(3,2);

            
            public static final TrapezoidProfile.Constraints strafeConstraints = 
                new TrapezoidProfile.Constraints(3,2);

            
            public static final TrapezoidProfile.Constraints rotationConstraints = 
                new TrapezoidProfile.Constraints(1,1);

            public static double driveOffset = 0.08;
            public static double strafeOffset = -5.16;
            public static double rotationOffset = 17;
            
            public static ProfiledPIDController drivePID = new ProfiledPIDController(kPdrive, kIdrive, kDdrive, driveConstraints);
            public static ProfiledPIDController strafePID = new ProfiledPIDController(kPstrafe, kIstrafe, kDstrafe, strafeConstraints);
            public static ProfiledPIDController rotationPID = new ProfiledPIDController(kProtation, kIrotation, kDrotation, rotationConstraints);


            public static final alignConstraints offsets =  
        new alignConstraints(driveOffset, strafeOffset, rotationOffset, drivePID, strafePID, rotationPID);

        }

        
    }

    public static final class PivotingConstants {

        public static final int kLeftMotorID = 10;
        public static final int kRightMotorID = 9;

        public static final int kMotorsCurrentLimit = 35;

        public static final boolean kLeftMotorInverted = true;
        public static final boolean kRightMotorInverted = false;

        public static final double kP = 1.35;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kMaxVelocity = 22;
        public static final double kMaxAcceleration = 22;

        public static final double kPIDminInput = 0;
        public static final double kPIDMaxInput = 0.995;

    }

    public static final class IntakeConstants {
    
        public static final int kMotorID = 11; 
        public static final boolean kMotorInverted = false;
        public static final IdleMode kMotorIdleMode = IdleMode.kBrake;
        
        public static final int kIRSensor1ID = 3;
        public static final int kIRSensor2ID = 0;
        
    }

    public static final class ShooterConstants {

        public static final int kRightMotorID = 12;
        public static final int kLeftMotorID = 13;

        public static final boolean kRightMotorInverted = true;
        public static final boolean kLeftMotorInverted = true;

        public static final IdleMode kMotorsIdleMode = IdleMode.kBrake;

    }

   

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kPlacerControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.08;
    }

    
    public static final class PS4OIConstants {
        
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;

        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 4;

        public static final double kDeadband = 0.05;
    }
    

    public static final class Shuffleboard {

        public static final ShuffleboardTab kShuffleboardTab = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Imperator");
    
        }

}