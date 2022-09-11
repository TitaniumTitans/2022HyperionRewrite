// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.titaniumtitans.frc2022.Constants.DriveConstants;
import org.titaniumtitans.frc2022.Constants.ModuleConstants;
import org.titaniumtitans.lib.Utils;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Robot swerve modules
    private final SwerveModuleNew m_frontLeft = new SwerveModuleNew(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftTurningEncoderPorts,
            //360 - 45.35,
            181,
            "FL",
            false);

    private final SwerveModuleNew m_rearLeft = new SwerveModuleNew(
            DriveConstants.kRearLeftDriveMotorPort,
            DriveConstants.kRearLeftTurningMotorPort,
            DriveConstants.kRearLeftTurningEncoderPorts,
            225,
            //0,
            "RL",
            true);

    private final SwerveModuleNew m_frontRight = new SwerveModuleNew(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightTurningEncoderPorts,
            170,
            //0,
            "FR",
            false);

    private final SwerveModuleNew m_rearRight = new SwerveModuleNew(
            DriveConstants.kRearRightDriveMotorPort,
            DriveConstants.kRearRightTurningMotorPort,
            DriveConstants.kRearRightTurningEncoderPorts,
            71,
            //0,
            "RR",
            false);

    private final SwerveModuleNew[] m_modules = new SwerveModuleNew[]{m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};

    // The gyro sensor
    private final Gyro m_gyro = new WPI_PigeonIMU(15);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

    private boolean fieldRelative = false;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        ShuffleboardTab debugTab = Shuffleboard.getTab("Drivetrain");
        debugTab.add("SwerveState", new SwerveModuleSendable());
        for(SwerveModuleNew module : m_modules) {
            //debugTab.add(module.getName() + " Module", module);
        }
    }

    private final class SwerveModuleSendable implements Sendable {

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");
            for (SwerveModuleNew module : m_modules) {
                
                builder.addDoubleProperty(module.getName() + "/CurrentStateAngle", () -> module.getState().angle.getDegrees(), null);
                builder.addDoubleProperty(module.getName() + "/CurrentStateSpeed", () -> module.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty(module.getName() + "/DesiredStateAngle", () -> module.getDesiredState().angle.getDegrees(), null);
                builder.addDoubleProperty(module.getName() + "/DesiredStateSpeed", () -> module.getDesiredState().speedMetersPerSecond, null);
                builder.addDoubleProperty(module.getName() + "/DrivePercentage", module::getDrivePercentage, null);
                builder.addDoubleProperty(module.getName() + "/TurningPercentage", module::getAzimuthPercentage, null);
                
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

        SmartDashboard.putBoolean("Field Oriented?", fieldRelative);

        
        SmartDashboard.putNumber("Encoder" + m_frontLeft.getName(), m_frontLeft.getAbsolutePosition());
        SmartDashboard.putNumber("Encoder" + m_frontRight.getName(), m_frontRight.getAbsolutePosition());
        SmartDashboard.putNumber("Encoder" + m_rearLeft.getName(), m_rearLeft.getAbsolutePosition());
        SmartDashboard.putNumber("Encoder" + m_rearRight.getName(), m_rearRight.getAbsolutePosition());
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
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

                        
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setModuleState(swerveModuleStates[0]);
        m_frontRight.setModuleState(swerveModuleStates[1]);
        m_rearLeft.setModuleState(swerveModuleStates[2]);
        m_rearRight.setModuleState(swerveModuleStates[3]);
    }


    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        for (SwerveModuleNew m_module : m_modules) {
            m_module.resetEncoders();
        }
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

    /*
    public void setModuleAngle(double angle){
        for(SwerveModuleNew module: m_modules){
            module.setModuleAngle(angle);
        }
    }
    */

    public void setTestState(double angle){
        SwerveModuleState state = createTestState(angle);

        for(SwerveModuleNew module: m_modules){
            module.setModuleState(state);
        }
    }

    public SwerveModuleState createTestState(double angle){
        return new SwerveModuleState(0, Rotation2d.fromDegrees(angle));
    }
}
