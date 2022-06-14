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
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD:src/main/java/org/titaniumtitans/frc2022/subsystems/SwerveDrive.java
import org.titaniumtitans.frc2022.Constants.SwerveConstants;


public class SwerveDrive extends SubsystemBase {

    private final SwerveDriveModule m_frontLeft = 
        new SwerveDriveModule(SwerveConstants.frontLeftDrive, SwerveConstants.frontLeftTurning, SwerveConstants.frontLeftEncoder, 347.695);
    
    private final SwerveDriveModule m_frontRight =
        new SwerveDriveModule(SwerveConstants.frontRightDrive, SwerveConstants.frontRightTurning, SwerveConstants.frontRightEncoder, 359.297);

    private final SwerveDriveModule m_backLeft = 
        new SwerveDriveModule(SwerveConstants.backLeftDrive, SwerveConstants.backLeftTurning, SwerveConstants.backLeftEncoder, 251.104);

    private final SwerveDriveModule m_backRight =
        new SwerveDriveModule(SwerveConstants.backRightDrive, SwerveConstants.backRightTurning, SwerveConstants.backRightEncoder, 227.5488);

    private final PigeonIMU m_gyro = new PigeonIMU(SwerveConstants.gyroPort);

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(SwerveConstants.kSwerveKinematics, new Rotation2d(m_gyro.getAbsoluteCompassHeading()));

    /** Creates a new SwerveDrive. */
    public SwerveDrive() {}

    @Override
    public void periodic() {
        //Update the odometry
        m_odometry.update(
            new Rotation2d(m_gyro.getCompassHeading()),
                m_frontLeft.getState(),
                m_frontRight.getState(), 
                m_backLeft.getState(), 
                m_backRight.getState());
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        m_odometry.resetPosition(pose, new Rotation2d(m_gyro.getAbsoluteCompassHeading()));
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
        var swerveModuleStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(m_gyro.getAbsoluteCompassHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public void zeroHeading(){
        m_gyro.setCompassAngle(0);
    }

    public double getHeading(){
        return m_gyro.getAbsoluteCompassHeading();
    }

    public double getTurnRate(){
        double dps[] = new double[3];
        m_gyro.getRawGyro(dps);
        return dps[2];
    }
=======

public class SwerveDrive extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          SwerveConstants.frontLeftDrive,
          SwerveConstants.frontLeftTurning,
          SwerveConstants.frontLeftEncoder,
          11.07);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          SwerveConstants.backLeftDrive,
          SwerveConstants.backLeftTurning,
          SwerveConstants.backLeftEncoder,
          337.412);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          SwerveConstants.frontRightDrive,
          SwerveConstants.frontRightTurning,
          SwerveConstants.frontRightEncoder,
          345.673);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          SwerveConstants.backRightDrive,
          SwerveConstants.backRightTurning,
          SwerveConstants.backRightEncoder,
          23.291);

  // The gyro sensor
  private final Gyro m_gyro = new WPI_PigeonIMU(15);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(SwerveConstants.kSwerveKinematics, m_gyro.getRotation2d());

  private double lastX = 0;
  private double lastY = 0;
  private double lastRot = 0;

  /** Creates a new DriveSubsystem. */
  public SwerveDrive() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());

        //Update encoder values on Shuffleboard
        SmartDashboard.putNumber("FR Angle", m_frontRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("FL Angle", m_frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("BR Angle", m_rearRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("BL Angle", m_rearLeft.getState().angle.getDegrees());
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
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param SwerveModuleState 
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    if(xSpeed != 0.0 && ySpeed != 0.0 && rot != 0.0){
    swerveModuleStates =
        SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    } else {
      swerveModuleStates =
        SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(lastX, lastY, lastRot, m_gyro.getRotation2d())
                : new ChassisSpeeds(lastX, lastY, lastRot));
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    lastX = xSpeed;
    lastY = ySpeed;
    lastRot = rot;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
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
    return m_gyro.getRate();
  }
>>>>>>> main:2022HyperionRewrite/src/main/java/frc/robot/subsystems/SwerveDrive.java
}
