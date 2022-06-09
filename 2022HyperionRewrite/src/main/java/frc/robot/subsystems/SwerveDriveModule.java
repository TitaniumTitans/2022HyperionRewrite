// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveDriveModule extends SubsystemBase {
  private static TalonSRX m_turning;
  private static TalonSRX m_drive;
  private static CANCoder m_encoder;
  /** Creates a new SwerveDriveModule. */
  public SwerveDriveModule(int drivePort, int turningPort, int encoderPort, boolean encoderReversed) {
    m_turning = new TalonSRX(turningPort);
    // configure onboard pid loop for turning motor
    m_turning.config_kP(1, ModuleConstants.kPTurn);
    m_turning.config_kI(1, ModuleConstants.kITurn);
    m_turning.config_kD(1, ModuleConstants.kDTurn);
    m_turning.config_IntegralZone(1, ModuleConstants.kIzone);

    // configure motion profile for turning motor
    m_turning.configMotionAcceleration(ModuleConstants.kMaxModuleAccelerationSpeedDegreesPerSecond);
    m_turning.configMotionCruiseVelocity(ModuleConstants.kMaxModuleAngularSpeedDegreesPerSecond);
    m_turning.configMotionSCurveStrength(0);

    m_drive = new TalonSRX(drivePort);
    m_drive.config_kF(1, ModuleConstants.kPDrive);

    m_encoder = new CANCoder(encoderPort);
    m_encoder.configSensorDirection(encoderReversed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
