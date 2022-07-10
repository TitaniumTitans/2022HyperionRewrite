// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.titaniumtitans.frc2022.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Robot swerve modules
    private final SwerveModule m_frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftTurningEncoderPorts,
            0,
            "FL");

    private final SwerveModule m_rearLeft = new SwerveModule(
            DriveConstants.kRearLeftDriveMotorPort,
            DriveConstants.kRearLeftTurningMotorPort,
            DriveConstants.kRearLeftTurningEncoderPorts,
            0,
            "RL");

    private final SwerveModule m_frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightTurningEncoderPorts,
            0,
            "FR");

    private final SwerveModule m_rearRight = new SwerveModule(
            DriveConstants.kRearRightDriveMotorPort,
            DriveConstants.kRearRightTurningMotorPort,
            DriveConstants.kRearRightTurningEncoderPorts,
            0,
            "RR");

    private final SwerveModule[] m_modules = new SwerveModule[]{m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};

    // The gyro sensor
    private final Gyro m_gyro = new WPI_PigeonIMU(15);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

    private boolean fieldRelative = false;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        Shuffleboard.getTab("Drivetrain").add("SwerveState", new SwerveModuleSendable());
    }

    private final class SwerveModuleSendable implements Sendable {

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");
            for (SwerveModule module : m_modules) {
                builder.addDoubleProperty(module.getName() + "/CurrentStateAngle", () -> module.getState().angle.getDegrees(), null);
                builder.addDoubleProperty(module.getName() + "/CurrentStateSpeed", () -> module.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty(module.getName() + "/DesiredStateAngle", () -> module.getDesiredState().angle.getDegrees(), null);
                builder.addDoubleProperty(module.getName() + "/DesiredStateSpeed", () -> module.getDesiredState().speedMetersPerSecond, null);
                builder.addDoubleProperty(module.getName() + "/DrivePercentage", module::getDriveMotorPercentage, null);
                builder.addDoubleProperty(module.getName() + "/TurningPercentage", module::getTurningMotorPercentage, null);
            }
        }
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                m_gyro.getRotation2d(),
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());

        SmartDashboard.putNumber("FR Angle", m_frontRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("FL Angle", m_frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("BR Angle", m_rearRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("BL Angle", m_rearLeft.getState().angle.getDegrees());
        SmartDashboard.putBoolean("Field Oriented?", fieldRelative);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed            Speed of the robot in the x direction (forward).
     * @param ySpeed            Speed of the robot in the y direction (sideways).
     * @param rot               Angular rate of the robot.
     * @param fieldRelative     Whether the provided x and y speeds are relative to
     *                          the field.
     * @param SwerveModuleState
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);

    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void changeDriveMode(){
        fieldRelative = !fieldRelative;
    }
}
