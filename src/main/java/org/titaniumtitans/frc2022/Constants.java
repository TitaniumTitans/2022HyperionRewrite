// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * Constants for the SwerveDrive subsystem. This includes CAN IDs, track
     * width/length, maximum velocity, and kinematics.
     */
    public static class SwerveConstants {
        public static final int frontLeftDrive = 3;
        public static final int frontRightDrive = 6;
        public static final int backLeftDrive = 12;
        public static final int backRightDrive = 9;

        public static final int frontLeftTurning = 4;
        public static final int frontRightTurning = 7;
        public static final int backLeftTurning = 13;
        public static final int backRightTurning = 10;

        public static final int frontLeftEncoder = 5;
        public static final int frontRightEncoder = 8;
        public static final int backLeftEncoder = 14;
        public static final int backRightEncoder = 11;

        public static final int gyroPort = 15;

        public static final double kTrackWidth = 20.733;
        // Distance between the left and rigth wheels
        public static final double kWheelBase = 20.733;
        // Distance between the front and back wheels
        // With a square robot, these numbers are the same

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kMaxSpeedMetersPerSecond = 3;
    }

    /**
     * Constants for each swerve module. This includes encoder CPRs, maximum
     * speed/accelerations, and PID constants.
     */
    public static class ModuleConstants {
        public static final int kTurningEncoderCPR = 4096;
        public static final int kDriveEncoderCPR = 2048;
        
        public static final double kMaxModuleAngularSpeedDegreesPerSecond = 5000;
        public static final double kMaxModuleAccelerationSpeedDegreesPerSecond = 5000;
        
        public static final double kWheelDiameterMeter = 0.0381;
        
        public static final double kTurningDistancePerPulse = (2 * Math.PI) / (double) kTurningEncoderCPR;
        public static final double kDriveDistancePerPulse = (kWheelDiameterMeter * Math.PI) / 8.14
        / (double) kDriveEncoderCPR;
        
        public static final double kPDrive = 1;
        
        public static final double kPTurn = 0.8;
        public static final double kITurn = 0.001;
        public static final double kDTurn = 20;
        public static final double kIzone = 20;
        public static final PIDController driveController = new PIDController(kPDrive, 0.0, 0.0);
    }

    /**
     * Currently unused, but will eventually contain any constants relative to the
     * autonomous period.
     */
    public static class AutoConstants {
    }

    /**
     * Currently unused, but will eventually contain constants used by the Shooter subsystem.
     */
    public static class ShooterConstants {
    }

    /**
     * CAN IDs and sensor DIO ports for the indexer subsystem.
     */
    public static class IndexerConstants {
        public static final int compressorPort = 1;
        public static final int intakeOpenPort = 4;
        public static final int intakeClosePort = 5;

        public static final int intakeDriveID = 16;
        public static final int magazineDriveID = 17;
        public static final int kickerDriveID = 18;

        public static final int lowSensorDIOPort = 0;

        public static final int highSensorDIOPort = 1;
    }

    /** 
     * Currently unused, but will eventually contain constants for the Climber subsystem.
     */
    public static class ClimberConstants {
    }

    /** 
     * Currently unused, but will eventually contain constants for the Turret subsystem.
     */
    public static class TurretConstants {
    }
}