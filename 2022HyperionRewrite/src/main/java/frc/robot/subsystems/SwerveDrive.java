// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;


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
}
