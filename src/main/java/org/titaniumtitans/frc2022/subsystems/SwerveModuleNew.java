// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.titaniumtitans.frc2022.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.titaniumtitans.lib.Utils;
import org.titaniumtitans.lib.Swerve.CTREModuleState;
import org.titaniumtitans.lib.Swerve.SwerveAzimuthFactoy;

public class SwerveModuleNew extends SubsystemBase {
  private final TalonFX m_azimuth;
  private final TalonFX m_drive;
  private final CANCoder m_encoder;
  /** Creates a new SwerveModuleNew. */
  public SwerveModuleNew(int drivePort, int azimuthPort, int encoderPort) {
    m_azimuth = SwerveAzimuthFactoy.createAzimuthTalon(azimuthPort);
    m_drive = new TalonFX(drivePort);
    m_encoder = new CANCoder(encoderPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Rotation2d getAzimuthAngle(){
    return Rotation2d.fromDegrees(Utils.falconToDegrees(m_azimuth.getSelectedSensorPosition(), ModuleConstants.kTurningGearRatio));
  }

  public void setModuleState(SwerveModuleState state){
    SwerveModuleState desiredState = CTREModuleState.optimize(state, getAzimuthAngle());

    double driveOutput = Utils.MPSToFalcon(desiredState.speedMetersPerSecond, ModuleConstants.kWheelDiameterMeters * Math.PI, ModuleConstants.kDriveGearRatio);

    double turningOutput = Utils.degreesToFalcon(desiredState.angle.getDegrees(), ModuleConstants.kTurningGearRatio);

    if(SmartDashboard.getBoolean("Enable Driving", true)){
      m_drive.set(ControlMode.Velocity, driveOutput);
      m_azimuth.set(ControlMode.Position, turningOutput);
    }
  }

  public SwerveModuleState getModuleState(){
    return new SwerveModuleState(m_drive.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderDistancePerPulse * 10,  
        getAzimuthAngle());
  }

  public void resetEncoders(){
    m_azimuth.setSelectedSensorPosition(0);
    m_drive.setSelectedSensorPosition(0);
  }

  public void setAbsoluteValue(){
    double absolutePosition = Utils.degreesToFalcon(m_encoder.getAbsolutePosition(), ModuleConstants.kTurningGearRatio);
        m_azimuth.setSelectedSensorPosition(absolutePosition);
    }
}
