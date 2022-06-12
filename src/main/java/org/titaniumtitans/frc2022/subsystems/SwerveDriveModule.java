// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.titaniumtitans.frc2022.Constants;
import org.titaniumtitans.frc2022.Constants.ModuleConstants;;

public class SwerveDriveModule extends SubsystemBase {
  private static TalonSRX m_turning;
  private static TalonSRX m_drive;
  private static CANCoder m_encoder;
  /** Creates a new SwerveDriveModule. */
  public SwerveDriveModule(int drivePort, int turningPort, int encoderPort, double encoderOffset) {
    m_turning = new TalonSRX(turningPort);
    // configure onboard pid loop for turning motor
    m_turning.config_kP(1, ModuleConstants.kPTurn);
    m_turning.config_kI(1, ModuleConstants.kITurn);
    m_turning.config_kD(1, ModuleConstants.kDTurn);
    m_turning.config_IntegralZone(1, ModuleConstants.kIzone);

    m_turning.setNeutralMode(NeutralMode.Brake);

    // configure motion profile for turning motor
    m_turning.configMotionAcceleration(ModuleConstants.kMaxModuleAccelerationSpeedDegreesPerSecond);
    m_turning.configMotionCruiseVelocity(ModuleConstants.kMaxModuleAngularSpeedDegreesPerSecond);
    m_turning.configMotionSCurveStrength(0);

    m_turning.configRemoteFeedbackFilter(encoderPort, RemoteSensorSource.CANCoder, 0);

    m_drive = new TalonSRX(drivePort);
    m_drive.config_kF(1, ModuleConstants.kPDrive);
    m_drive.setNeutralMode(NeutralMode.Brake);
    m_drive.configOpenloopRamp(0.5);
    m_drive.configClosedloopRamp(0.5);

    m_encoder = new CANCoder(encoderPort);
    m_encoder.configFactoryDefault();
    m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    m_encoder.configMagnetOffset(encoderOffset);
    m_encoder.configSensorDirection(true);
    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(m_drive.getSelectedSensorVelocity(), new Rotation2d(m_turning.getSelectedSensorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    // Optimize the swerve module state so the wheel doesn't turn more than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turning.getSelectedSensorPosition()));

    final double drivingOutput = (state.speedMetersPerSecond / (Constants.ModuleConstants.kWheelDiameterMeter / 2)) * 321.6990877275948;

    final double turningOutput = state.angle.getDegrees();

    m_drive.set(ControlMode.Velocity, drivingOutput);
    m_drive.set(ControlMode.Position, turningOutput);
  }

  public void resetEncoders(){
    m_drive.setSelectedSensorPosition(0);
    m_turning.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
