// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.titaniumtitans.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.gos.lib.ctre.CtrePidPropertyBuilder;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.PidProperty;
import org.titaniumtitans.frc2022.Constants.ClimberConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX m_leftClimber;
  private final TalonFX m_rightClimber;

  public final GosDoubleProperty m_rightMaxHeight;
  public final GosDoubleProperty m_leftMaxHeight;

  private final PidProperty m_leftPidProperty;
  private final PidProperty m_rightPidProperty;

  private double m_lastPower;
  private boolean m_direction;

  /** Creates a new Climber. */
  public Climber() {
    m_leftClimber = new TalonFX(ClimberConstants.kLeftCimberPort);
    m_rightClimber = new TalonFX(ClimberConstants.kRightClimberPort);

    m_rightMaxHeight = new GosDoubleProperty(false, "Right Max Height", 0.0);
    m_leftMaxHeight = new GosDoubleProperty(false, "Left Max Height", 0.0);

    m_rightClimber.setInverted(InvertType.InvertMotorOutput);

    m_leftPidProperty = new CtrePidPropertyBuilder("Left CLimber", true, m_leftClimber, 0)
      .addP(0.45)
      .addD(0.0)
      .addMaxAcceleration(40000)
      .addMaxVelocity(40000)
      .build();

      m_rightPidProperty = new CtrePidPropertyBuilder("Right CLimber", true, m_rightClimber, 0)
      .addP(0.45)
      .addD(0.0)
      .addMaxAcceleration(40000)
      .addMaxVelocity(40000)
      .build(); 

      m_lastPower = 0.0;
      m_direction = true;
      
    SmartDashboard.putNumber("Climber Speed", 0.0);

    //TODO Implement the climber brakes
    
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber Height", m_leftClimber.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Climber Height", m_rightClimber.getSelectedSensorPosition());
    SmartDashboard.putNumber("Output power", m_lastPower);
    SmartDashboard.putBoolean("Is going up?", m_direction);

    m_leftPidProperty.updateIfChanged();
    m_rightPidProperty.updateIfChanged();
  }

  public void resetEncoders(){
    m_rightClimber.setSelectedSensorPosition(0);
    m_leftClimber.setSelectedSensorPosition(0);
  }

  public void joystickControl(double speed, boolean isUp){
    m_lastPower = speed;
    m_direction = isUp;
    m_leftClimber.set(ControlMode.PercentOutput, speed);
    m_rightClimber.set(ControlMode.PercentOutput, speed);
  }

  public void climberPidControl(double leftPosition, double rightPosition){
    m_leftClimber.set(ControlMode.MotionMagic, leftPosition);
    m_rightClimber.set(ControlMode.MotionMagic, rightPosition);
  }

}
