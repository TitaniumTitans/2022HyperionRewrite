// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.titaniumtitans.frc2022.Constants.ModuleConstants;
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
}
