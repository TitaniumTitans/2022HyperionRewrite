// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class SwerveConstants{
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
    }

    public static class ModuleConstants{
        public static final int kTurningEncoderCPR = 4096;
        public static final int kDriveEncoderCPR = 2048;

        public static final double kWheelDiameterMeter = 0.0381;

        public static final double kTurningDistancePerPulse = (2 * Math.PI) / (double) kTurningEncoderCPR;
        public static final double kDriveDistancePerPulse = (kWheelDiameterMeter * Math.PI) / 8.14 / (double) kDriveEncoderCPR;
    }

    public static class AutoConstants{}

    public static class ShooterConstants{}

    public static class IndexerConstants{
        public static final int compressorPort = 1;
        public static final int intakeOpenPort = 4;
        public static final int intakeClosePort = 5;

        public static final int intakeDriveID = 16;
        public static final int magazineDriveID = 17;
        public static final int kickerDriveID = 18;

        public static final int lowSensorDIOPort = 1;
        public static final int highSensorDIOPort = 2;
    }

    public static class ClimberConstants{}

    public static class TurretConstants{}
}
