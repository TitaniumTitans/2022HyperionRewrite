// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.gos.lib.properties.PidProperty;
import org.gos.lib.properties.PropertyManager;
import org.gos.lib.properties.CTRE.CtrePidPropertyBuilder;
import org.gos.lib.properties.PropertyManager.IProperty;
import org.titaniumtitans.frc2022.Constants.ClimberConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX m_leftClimber;
  private final TalonFX m_rightClimber;

  private final IProperty<Double> m_rightMaxHeight;
  private final IProperty<Double> m_leftMaxHeight;

  private final PidProperty m_leftPidProperty;
  private final PidProperty m_rightPidProperty;

  /** Creates a new Climber. */
  public Climber() {
    m_leftClimber = new TalonFX(ClimberConstants.kLeftCimberPort);
    m_rightClimber = new TalonFX(ClimberConstants.kRightClimberPort);

    m_rightMaxHeight = PropertyManager.createDoubleProperty(false, "Right Max Height", 0.0);
    m_leftMaxHeight = PropertyManager.createDoubleProperty(false, "Left Max Height", 0.0);

    m_leftPidProperty = new CtrePidPropertyBuilder("Left CLimber", false, m_leftClimber, 0)
      .addP(0.0)
      .addD(0.0)
      .addMaxAcceleration(1000)
      .addMaxVelocity(1000)
      .build();

      m_rightPidProperty = new CtrePidPropertyBuilder("Left CLimber", false, m_leftClimber, 0)
      .addP(0.0)
      .addD(0.0)
      .addMaxAcceleration(1000)
      .addMaxVelocity(1000)
      .build();    
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber Height", m_leftClimber.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Climber Height", m_rightClimber.getSelectedSensorPosition());
  }

  public void resetEncoders(){
    m_rightClimber.setSelectedSensorPosition(0);
    m_leftClimber.setSelectedSensorPosition(0);
  }

  public void joystickControl(double speed){
    m_leftClimber.set(ControlMode.PercentOutput, speed);
    m_rightClimber.set(ControlMode.PercentOutput, speed);
  }

  public void climberPidControl(double position){
    m_leftClimber.set(ControlMode.MotionMagic, position);
    m_rightClimber.set(ControlMode.MotionMagic, position);
  }

}
