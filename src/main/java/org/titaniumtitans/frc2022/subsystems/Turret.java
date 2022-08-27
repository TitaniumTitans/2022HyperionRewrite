// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.titaniumtitans.frc2022.Constants.TurretConstants;
import org.titaniumtitans.lib.Utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Turret extends ProfiledPIDSubsystem {
  private final TalonSRX m_turretMotor;
  private final ShooterLimelight m_limeLight;

  /** Creates a new Turret. */
  public Turret() {
    super(
      // The PIDController used by the subsystem
      new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(TurretConstants.kCruiseVelocity, TurretConstants.kAcceleration)), 0);
      m_turretMotor = new TalonSRX(TurretConstants.kTurretPort);
      m_limeLight = new ShooterLimelight();
    }
    
  @Override
  protected void useOutput(double output, State setpoint) {
  // Use the output here
  double falconAngle = Utils.degreesToFalcon(output, TurretConstants.kTurretGearRatio);

  m_turretMotor.set(ControlMode.Position, falconAngle); 
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_turretMotor.getSelectedSensorPosition();
  }

  public double getPositionDeg(){
    return Utils.falconToDegrees(m_turretMotor.getSelectedSensorPosition(), TurretConstants.kTurretGearRatio);
  }

  public double getTargetOffset(){
    return m_limeLight.getTX();
  }

}
