// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final class DriveConstants {
        public static final int kFrontLeftDriveMotorPort = 3;
        public static final int kRearLeftDriveMotorPort = 12;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kRearRightDriveMotorPort = 9;

        public static final int kFrontLeftTurningMotorPort = 4;
        public static final int kRearLeftTurningMotorPort = 13;
        public static final int kFrontRightTurningMotorPort = 7;
        public static final int kRearRightTurningMotorPort = 10;

        public static final int kFrontLeftTurningEncoderPorts = 5;
        public static final int kRearLeftTurningEncoderPorts = 14;
        public static final int kFrontRightTurningEncoderPorts = 8;
        public static final int kRearRightTurningEncoderPorts = 11;

        public static final double kTrackWidth = Units.inchesToMeters(20.733);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(20.733);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for
        // your robot.
        // public static final double ksVolts = 1;
        // public static final double kvVoltSecondsPerMeter = 0.8;
        // public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double kMaxSpeedMetersPerSecond = 3;
    }

    public static final class ModuleConstants {
        public final static int kTimeoutMs = 100;

        public static final double kTurningGearRatio = (50.0 / 14.0) * (60.0 / 10.0);
        public static final double kDriveGearRatio = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);

        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 12.20703125;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 12.20703125;

        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / 8.14 / (double) 2048;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) kEncoderCPR;

        public static final double kCTREToWPILibPIDFConstants = 0.63661977237;
        public static final double kRadPerSecondToRawPer100_4096CPR = ((1 / (20 * Math.PI)) * 4096) / 100;


        public static final double kPModuleTurningController = 0.8 / kTurningGearRatio;
        public static final double kIModuleTurningController = 0.001 / kTurningGearRatio;
        public static final double kDModuleTurningController = 10 / kTurningGearRatio;
        public static final double kIZoneModuleTurningController = 20;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    /**
     * Contains constants for the autonomous period.
     */
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Units
                .radiansPerSecondToRotationsPerMinute(Math.PI) / 60;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Units
                .radiansPerSecondToRotationsPerMinute(Math.PI) / 60;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    /**
     * Currently unused, but will eventually contain constants used by the Shooter
     * subsystem.
     */
    public static class ShooterConstants {
        public static final double kTargetHeight = 102.619; //inches
        public static final double kLimelightHeight = 39.7483595; //inches
        public static final double kLimelightAngle = 17.728393; //degrees
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
     * Currently unused, but will eventually contain constants for the Climber
     * subsystem.
     */
    public static class ClimberConstants {
    }

    /**
     * Currently unused, but will eventually contain constants for the Turret
     * subsystem.
     */
    public static class TurretConstants {
            public static final double kTurretGearRatio = 0.0;

            public static final int kTurretPort = 19;
    }
}
